/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

 /* Author: Boston Cleek
    Desc:   Grasp generator stage using deep learning based grasp synthesizers
 */

 #include <moveit/task_constructor/stages/deep_grasp_generator.h>
 #include <moveit/task_constructor/storage.h>
 #include <moveit/task_constructor/marker_tools.h>
 #include <rviz_marker_tools/marker_creation.h>

 #include <moveit/planning_scene/planning_scene.h>

 #include <Eigen/Geometry>
 #include <eigen_conversions/eigen_msg.h>

 namespace moveit {
 namespace task_constructor {
 namespace stages {

 DeepGraspPose::DeepGraspPose(ros::NodeHandle& nh, const std::string& name)
                : GeneratePose(name),
                  client_(nh, "sample_grasps", true) {
 	auto& p = properties();
 	p.declare<std::string>("eef", "name of end-effector");
 	p.declare<std::string>("object");
 	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");

 	p.declare<boost::any>("pregrasp", "pregrasp posture");
 	p.declare<boost::any>("grasp", "grasp posture");
 }

 void DeepGraspPose::init(const core::RobotModelConstPtr& robot_model) {
 	InitStageException errors;
 	try {
 		GeneratePose::init(robot_model);
 	} catch (InitStageException& e) {
 		errors.append(e);
 	}

 	const auto& props = properties();

 	// check angle_delta
 	if (props.get<double>("angle_delta") == 0.)
 		errors.push_back(*this, "angle_delta must be non-zero");

 	// check availability of object
 	props.get<std::string>("object");
 	// check availability of eef
 	const std::string& eef = props.get<std::string>("eef");
 	if (!robot_model->hasEndEffector(eef))
 		errors.push_back(*this, "unknown end effector: " + eef);
 	else {
 		// check availability of eef pose
 		const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
 		const std::string& name = props.get<std::string>("pregrasp");
 		std::map<std::string, double> m;
 		if (!jmg->getVariableDefaultPositions(name, m))
 			errors.push_back(*this, "unknown end effector pose: " + name);
 	}

 	if (errors)
 		throw errors;
 }

 void DeepGraspPose::onNewSolution(const SolutionBase& s) {
 	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

 	const auto& props = properties();
 	const std::string& object = props.get<std::string>("object");
 	if (!scene->knowsFrameTransform(object)) {
 		const std::string msg = "object '" + object + "' not in scene";
 		if (storeFailures()) {
 			InterfaceState state(scene);
 			SubTrajectory solution;
 			solution.markAsFailure();
 			solution.setComment(msg);
 			spawn(std::move(state), std::move(solution));
 		} else
 			ROS_WARN_STREAM_NAMED("GenerateGraspPose", msg);
 		return;
 	}

 	upstream_solutions_.push(&s);
 }

 void DeepGraspPose::compute() {
 	if (upstream_solutions_.empty()){
    return;
  }

  geometry_msgs::PoseStamped target_pose_msg;

  moveit_task_constructor_msgs::GenerateDeepGraspPoseGoal goal;
  goal.action_name = "generate_grasps";
  client_.sendGoal(goal);

  ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  std::cout << client_.isServerConnected() << std::endl;

  client_.waitForResult(ros::Duration(5.0));
  if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    printf("Done!");

    moveit_task_constructor_msgs::GenerateDeepGraspPoseResultConstPtr grasp_result_ptr;
    grasp_result_ptr = client_.getResult();

    target_pose_msg = grasp_result_ptr->grasp_candidate;

  }
  printf("Current State: %s\n", client_.getState().toString().c_str());

  ROS_WARN("!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  std::cout << target_pose_msg << std::endl;

  // target_pose_msg.header.frame_id = "panda_link0";
  // target_pose_msg.pose.position.x = 0.5;
  // target_pose_msg.pose.position.y = -0.25;
  // target_pose_msg.pose.position.z = 0.20;


 	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

 	// set end effector pose
 	const auto& props = properties();
 	const std::string& eef = props.get<std::string>("eef");
 	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

 	robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
 	robot_state.setToDefaultValues(jmg, props.get<std::string>("pregrasp"));


	InterfaceState state(scene);
	state.properties().set("target_pose", target_pose_msg);
	props.exposeTo(state.properties(), { "pregrasp", "grasp" });

	SubTrajectory trajectory;
	trajectory.setCost(0.0);
	trajectory.setComment(std::to_string(0.0));

	// add frame at target pose
	rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

  spawn(std::move(state), std::move(trajectory));
 }
 }  // namespace stages
 }  // namespace task_constructor
 }  // namespace moveit
