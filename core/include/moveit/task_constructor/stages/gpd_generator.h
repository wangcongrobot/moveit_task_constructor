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

 #include <memory>

 #include <moveit_task_constructor_msgs/GenerateDeepGraspPoseAction.h>
 #include <actionlib/client/simple_action_client.h>
 // #include <actionlib/action_definition.h>

 namespace moveit {
 namespace task_constructor {
 namespace stages {

 template<class ActionSpec>
 class ActionStageBase
 {
 private:
   ACTION_DEFINITION(ActionSpec);

 public:
   ActionStageBase()
   {
     clientPtr_.reset(new actionlib::SimpleActionClient<ActionSpec>("topic_name", true));
   }
   // void feedbackCallback(const FeedbackConstPtr &feedback);

protected:
  std::unique_ptr<actionlib::SimpleActionClient<ActionSpec>> clientPtr_;
};


 class DeepGraspPose : public GeneratePose
 {
 public:
 	DeepGraspPose(const std::string& name = "generate grasp pose");

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

  actionlib::SimpleActionClient<moveit_task_constructor_msgs::GenerateDeepGraspPoseAction> client_;
 };
 }  // namespace stages
 }  // namespace task_constructor
 }  // namespace moveit
