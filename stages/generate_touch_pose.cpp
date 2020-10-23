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

#include "generate_touch_pose.h"
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model/aabb.h>
#include <Eigen/Geometry>

namespace moveit { namespace task_constructor { namespace stages {

GenerateTouchPose::GenerateTouchPose(const std::string& name)
   : GenerateGraspPose(name)
{
}

// TODO: move into MoveIt core
moveit::core::AABB getAABB(const std::vector<shapes::ShapeConstPtr>& shapes,
                           const EigenSTL::vector_Isometry3d& shape_poses) {
	core::AABB aabb;

	for (std::size_t i = 0; i < shapes.size(); ++i)
	{
		const Eigen::Isometry3d& transform = shape_poses[i];

		if (shapes[i]->type != shapes::MESH)
		{
			Eigen::Vector3d extents = shapes::computeShapeExtents(shapes[i].get());
			aabb.extendWithTransformedBox(transform, extents);
		}
		else
		{
			// we cannot use shapes::computeShapeExtents() for meshes, since that method does not provide information about
			// the offset of the mesh origin
			const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shapes[i].get());
			for (unsigned int j = 0; j < mesh->vertex_count; ++j)
				aabb.extend(transform * Eigen::Map<Eigen::Vector3d>(&mesh->vertices[3 * j]));
		}
	}
	return aabb;
}

Eigen::Vector3d touchPoint(const collision_detection::World::ObjectConstPtr &object) {
	core::AABB aabb = getAABB(object->shapes_, object->shape_poses_);
	Eigen::Vector3d result = aabb.center();
	result[2] += 0.5 * aabb.sizes()[2];
	return result;
}

void GenerateTouchPose::compute()
{
	if (upstream_solutions_.empty())
		return;
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	const std::string& eef = props.get<std::string>("eef");

	// enable object collision with end-effector (for ComputeIK)
	collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
	acm.setEntry(object, scene->getRobotModel()->getEndEffector(eef)
	             ->getLinkModelNamesWithCollisionGeometry(), true);

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = scene->getPlanningFrame();
	pose.pose.position = tf2::toMsg(touchPoint(scene->getWorld()->getObject(object)));

	double current_angle_ = 0.0;
	while (current_angle_ < 2.*M_PI && current_angle_ > -2.*M_PI) {
		// rotate object pose about z-axis
		pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ())));
		current_angle_ += props.get<double>("angle_delta");

		InterfaceState state(scene);
		state.properties().set("target_pose", pose);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setComment(std::to_string(current_angle_));

		// add frame at target pose
		rviz_marker_tools::appendFrame(trajectory.markers(), pose, 0.1, "touch frame");
		spawn(std::move(state), std::move(trajectory));
	}
}

} } }
