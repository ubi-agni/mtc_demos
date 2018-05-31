/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke */

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <actionlib/client/simple_action_client.h>
#include <grasping_msgs/GenerateGraspsAction.h>

#pragma once

namespace moveit { namespace task_constructor { namespace stages {

/** Call an actionlib server to provide grasps via grasping_msgs::GenerateGraspsAction
 */
class GraspProvider : public MonitoringGenerator {
public:
	GraspProvider(const std::string& name = "get grasps", const std::string& action_name = "grasp_provider");

	void reset() override;
	void init(const moveit::core::RobotModelConstPtr &robot_model) override;
	bool canCompute() const override;
	void compute() override;

	void setObject(const std::string &object) {
		setProperty("object", object);
	}

protected:
	void onNewSolution(const SolutionBase& s) override;

	// TODO: SimpleActionClient/Server cannot handle parallel requests!
	actionlib::SimpleActionClient<grasping_msgs::GenerateGraspsAction> ac_;
	std::list<std::pair<planning_scene::PlanningSceneConstPtr, grasping_msgs::GenerateGraspsGoal>> pending_;

private:
	planning_scene::PlanningSceneConstPtr sendNextRequest();
	moveit_msgs::RobotState posture(const trajectory_msgs::JointTrajectory& t);
};

} } }
