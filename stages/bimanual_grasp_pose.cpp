/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

/* Authors: Luca Lach, Robert Haschke */

#include "bimanual_grasp_pose.h"
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {


BimanualGraspPose::BimanualGraspPose(const std::string& name)
   : GeneratePose(name)
{
	auto& p = properties();
	p.declare<std::string>("object");
}

void BimanualGraspPose::init(const core::RobotModelConstPtr& robot_model)
{
	InitStageException errors;
	try { GeneratePose::init(robot_model); }
	catch (InitStageException &e) { errors.append(e); }

	// check availability of eefs
	if (eefs_.size() != 2)
		errors.push_back(*this, "need two end effectors");
	for (const auto& pair : eefs_) {
		if (!robot_model->hasEndEffector(pair.first))
			errors.push_back(*this, "unknown end effector: " + pair.first);
		else {
			// check availability of eef pose
			const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(pair.first);
			std::map<std::string, double> m;
			if (!jmg->getVariableDefaultPositions(pair.second, m))
				errors.push_back(*this, "unknown end effector pose: " + pair.second);
		}
	}

	if (errors) throw errors;
}

void BimanualGraspPose::onNewSolution(const SolutionBase& s)
{
	planning_scene::PlanningScenePtr scene = s.end()->scene()->diff();

	// set end effector poses
	for (const auto& pair : eefs_) {
		const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(pair.first);

		robot_state::RobotState &robot_state = scene->getCurrentStateNonConst();
		robot_state.setToDefaultValues(jmg , pair.second);
	}

	const std::string& object_name = properties().get<std::string>("object");
	collision_detection::World::ObjectConstPtr object = scene->getWorld()->getObject(object_name);
	if (!object) {
		ROS_WARN_STREAM_NAMED("BimanualGraspPose", "unknown object: " << object_name);
		return;
	}
	if (!object->shapes_.size()) {
		ROS_WARN_STREAM_NAMED("BimanualGraspPose", "Object " << object_name << "without shape");
		return;
	}

	scenes_.push_back(scene);
}

bool BimanualGraspPose::compute(){
	if (scenes_.empty())
		return false;
	planning_scene::PlanningSceneConstPtr scene = scenes_[0];
	scenes_.pop_front();

	const std::string& object_name = properties().get<std::string>("object");
	shapes::ShapeConstPtr shape = scene->getWorld()->getObject(object_name)->shapes_.front();

	geometry_msgs::PoseStamped target_pose;
	target_pose.header.frame_id = object_name;
	target_pose.pose.orientation.w = 1.0;

	double offset;
	switch(shape->type) {
	case(shapes::ShapeType::BOX):
		offset = std::static_pointer_cast<const shapes::Box>(shape)->size[1] / 2;
		break;
	case(shapes::ShapeType::SPHERE):
		offset = std::static_pointer_cast<const shapes::Sphere>(shape)->radius;
		break;
	case(shapes::ShapeType::CYLINDER):
		offset = std::static_pointer_cast<const shapes::Cylinder>(shape)->radius;
		break;
	default:
		ROS_WARN_STREAM_NAMED("BimanualGraspPose", "Object " << object_name << " has unsupported shape " << shape->type);
		return false;
	}
	moveit::task_constructor::InterfaceState state(scene);
	moveit::task_constructor::SubTrajectory trajectory;
	// right is shifted by +offset and turned 180° about z
	target_pose.pose.position.y = -offset;
	state.properties().set("target_pose_right", target_pose);
	rviz_marker_tools::appendFrame(trajectory.markers(), target_pose, 0.1, "grasp frame/right");

	// left is shifted by -offset
	target_pose.pose.position.y = +offset;
	target_pose.pose.orientation.z = 1.0;
	target_pose.pose.orientation.w = 0.0;
	state.properties().set("target_pose_left", target_pose);
	rviz_marker_tools::appendFrame(trajectory.markers(), target_pose, 0.1, "grasp frame/left");

	spawn(std::move(state), std::move(trajectory));
	return true;
}

} } }
