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

#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/action_base.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <memory>

#include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>


namespace moveit {
namespace task_constructor {
namespace stages {


template<class ActionSpec>
class DeepGraspPose : public GeneratePose, ActionBase<ActionSpec>
{
private:
	typedef ActionBase<ActionSpec> ActionBaseT;
	ACTION_DEFINITION(ActionSpec);

public:
	DeepGraspPose(const std::string& action_name,
		            const std::string& stage_name = "generate grasp pose");

	void activeCallback() override;

	void feedbackCallback(const FeedbackConstPtr &feedback) override;

	void doneCallback(const actionlib::SimpleClientGoalState& state,
                            const ResultConstPtr &result) override;

	void init(const core::RobotModelConstPtr& robot_model) override;
	void compute() override;

	void setEndEffector(const std::string& eef) { setProperty("eef", eef); }
	void setObject(const std::string& object) { setProperty("object", object); }

	void setPreGraspPose(const std::string& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setPreGraspPose(const moveit_msgs::RobotState& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setGraspPose(const std::string& grasp) { properties().set("grasp", grasp); }
	void setGraspPose(const moveit_msgs::RobotState& grasp) { properties().set("grasp", grasp); }

protected:
	void onNewSolution(const SolutionBase& s) override;

private:
	bool found_candidates_;
	std::vector<geometry_msgs::PoseStamped> grasp_candidates_;
	std::vector<double> costs_;
};

template<class ActionSpec>
DeepGraspPose<ActionSpec>::DeepGraspPose(const std::string& action_name,
	                                       const std::string& stage_name)
									:  GeneratePose(stage_name), ActionBaseT(action_name), found_candidates_(false) {
 auto& p = properties();
 p.declare<std::string>("eef", "name of end-effector");
 p.declare<std::string>("object");
 p.declare<double>("angle_delta", 0.1, "angular steps (rad)");

 p.declare<boost::any>("pregrasp", "pregrasp posture");
 p.declare<boost::any>("grasp", "grasp posture");

 ROS_INFO("Waiting for connection to grasp generation action server...");
 ActionBaseT::clientPtr_->waitForServer(ros::Duration(ActionBaseT::server_timeout_));
}


template<class ActionSpec>
void DeepGraspPose<ActionSpec>::activeCallback(){
	ROS_INFO("Generate grasp goal now active ");
	// found_candidates_ = false;
}


template<class ActionSpec>
void DeepGraspPose<ActionSpec>::feedbackCallback(const FeedbackConstPtr &feedback){
	// TODO: Check if costs are positive ?

	// each candidate should have a cost
	if (feedback->grasp_candidates.size() != feedback->costs.size()){
		ROS_ERROR("Invalid input: each grasp candidate needs an associated cost");
	}
	else{
		grasp_candidates_.resize(feedback->grasp_candidates.size());
		costs_.resize(feedback->costs.size());

		grasp_candidates_ = feedback->grasp_candidates;
		costs_ = feedback->costs;
	}
}


template<class ActionSpec>
void DeepGraspPose<ActionSpec>::doneCallback(const actionlib::SimpleClientGoalState& state,
													const ResultConstPtr &result){
	// TODO: add completion timeout

	if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
		found_candidates_ = true;
		ROS_INFO("Successfully found grasp candidates (result): %s", result->grasp_state.c_str());
	}
	else{
		ROS_WARN("No grasp candidates found (state): %s", ActionBaseT::clientPtr_->getState().toString().c_str());
	}
}


template<class ActionSpec>
void DeepGraspPose<ActionSpec>::init(const core::RobotModelConstPtr& robot_model){
	InitStageException errors;
	try {
		GeneratePose::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check angle_delta
	if (props.get<double>("angle_delta") == 0.){
		errors.push_back(*this, "angle_delta must be non-zero");
	}

	// check availability of object
	props.get<std::string>("object");
	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef)){
		errors.push_back(*this, "unknown end effector: " + eef);
	}
	else {
		// check availability of eef pose
		const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
		const std::string& name = props.get<std::string>("pregrasp");
		std::map<std::string, double> m;
		if (!jmg->getVariableDefaultPositions(name, m)){
			errors.push_back(*this, "unknown end effector pose: " + name);
		}
	}

	if (errors){
		throw errors;
	}
}


template<class ActionSpec>
void DeepGraspPose<ActionSpec>::compute(){


}


template<class ActionSpec>
void DeepGraspPose<ActionSpec>::onNewSolution(const SolutionBase& s){
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
		} else{
			ROS_WARN_STREAM_NAMED("GenerateGraspPose", msg);
		}
		return;
	}

	upstream_solutions_.push(&s);
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
