/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bielefeld University
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

#include "grasp_provider.h"
#include <grasping_msgs/GraspPlanningAction.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

GraspProvider::GraspProvider(const std::string& name, const std::string& action_name)
   : ac_(action_name, true)
{
}

void GraspProvider::reset()
{
	pending_.clear();
	MonitoringGenerator::reset();
}

void GraspProvider::init(const core::RobotModelConstPtr &robot_model)
{
	const auto& props = properties();
	props.get<std::string>("config");
	ac_.waitForServer(ros::Duration(timeout()));
}

void GraspProvider::onNewSolution(const SolutionBase& s)
{
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();
	const auto& props = properties();

	const std::string& object_name = props.get<std::string>("object");
	moveit_msgs::CollisionObject collision_object;
	if (!scene->getCollisionObjectMsg(collision_object, object_name)) {
		ROS_WARN_STREAM_NAMED("GraspProvider", "unknown object: " << object_name);
		return;
	}

	grasping_msgs::GenerateGraspsGoal goal;
	goal.object.header.frame_id = scene->getPlanningFrame();
	goal.object.name = object_name;
	goal.object.primitives = collision_object.primitives;
	goal.object.primitive_poses = collision_object.primitive_poses;
	goal.object.meshes = collision_object.meshes;
	goal.object.mesh_poses = collision_object.mesh_poses;

	goal.config_name = props.get<std::string>("config");

	// TODO: Here we could directly request a new goal and memorize the goal handle
	pending_.push_back(std::make_pair(scene, std::move(goal)));
}

bool GraspProvider::canCompute() const
{
	return pending_.size() > 0;
}

void GraspProvider::compute()
{
	auto scene = sendNextRequest();
	if (!scene) return;

	for (const moveit_msgs::Grasp& grasp : ac_.getResult()->grasps) {
		InterfaceState state(scene->diff());
		auto& props = state.properties();
		props.set("target_pose", grasp.grasp_pose);
		props.set("pregrasp", posture(grasp.pre_grasp_posture));
		props.set("grasp", posture(grasp.grasp_posture));
		props.set("pre_grasp_approach", grasp.pre_grasp_approach);
		props.set("post_grasp_retreat", grasp.post_grasp_retreat);

		SubTrajectory solution;
		solution.setCost(grasp.grasp_quality);
		solution.setComment(grasp.id);
		spawn(std::move(state), std::move(solution));
	}
}

moveit_msgs::RobotState GraspProvider::posture(const trajectory_msgs::JointTrajectory& t)
{
	if (t.points.size() != 1)
		throw std::runtime_error("expecting a single joint pose for pregrasp posture, got: " +
	                            std::to_string(t.points.size()));
	moveit_msgs::RobotState state;
	state.is_diff = true;
	state.joint_state.name = t.joint_names;
	state.joint_state.position = t.points[0].positions;
	return state;
}

planning_scene::PlanningSceneConstPtr GraspProvider::sendNextRequest()
{
	if (pending_.empty())
		return planning_scene::PlanningSceneConstPtr();

	// fetch first element from pending_ list
	decltype(pending_) target;
	target.splice(target.begin(), pending_, pending_.begin());

	ros::Duration timeout(this->timeout());
	// TODO: replace by asynchronous approach
	auto result = ac_.sendGoalAndWait(target.front().second, timeout, timeout);
	if (result != actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_WARN_STREAM_NAMED("GraspProvider", "actionlib server returned failed: " << result.getText());
		return planning_scene::PlanningSceneConstPtr();
	}
	return target.front().first;
}

} } }
