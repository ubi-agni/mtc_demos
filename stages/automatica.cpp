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

#include "automatica.h"
#include "generate_touch_pose.h"
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace moveit { namespace task_constructor { namespace stages {

Task* approachAndPush(const std::string& name, const std::string& side)
{
	std::string arm_group = side + "_arm";
	std::string hand_group = side + "_hand";
	std::string eef = side.substr(0,1) + "a_tool_mount";
	std::string tool_frame = side.substr(0,1) + "h_tool_frame";

	Task* task = new Task(name);
	Task& t = *task;

	Stage* initial = new stages::CurrentState("current");
	t.add(std::unique_ptr<Stage>(initial));

	// planners
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	pipeline->properties().set("max_velocity_scaling_factor", 0.1);

	auto interpolate = std::make_shared<solvers::JointInterpolationPlanner>();
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->properties().set("max_velocity_scaling_factor", 0.1);

	{
		// connect
		stages::Connect::GroupPlannerVector planners = {{hand_group, interpolate}, {arm_group, pipeline}};
		auto connect = new stages::Connect("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		connect->properties().set("merge_mode", stages::Connect::SEQUENTIAL);
		t.add(std::unique_ptr<Stage>(connect));
	}

	{  // allow touching of object
		auto allow_touch = new ModifyPlanningScene("allow object collision");
		PropertyMap& p = allow_touch->properties();
		p.set("eef", eef);
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT, { "object" });

		allow_touch->setCallback([](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
			collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
			const std::string& eef = p.get<std::string>("eef");
			const std::string& object = p.get<std::string>("object");
			acm.setEntry(object, scene->getRobotModel()->getEndEffector(eef)
			             ->getLinkModelNamesWithCollisionGeometry(), true);
		});
		t.add(std::unique_ptr<Stage>(allow_touch));
	}

	{  // approach
		auto approach = new stages::MoveRelative("approach", cartesian);
		approach->setGroup(arm_group);
		approach->properties().set("marker_ns", std::string("approach"));
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -1.0;
		approach->setGoal(direction);
		approach->setMinMaxDistance(0.03, 0.05);
		t.add(std::unique_ptr<Stage>(approach));
	}

	{  // touch
		auto approach = new stages::MoveRelative("touch", cartesian);
		approach->setGroup(arm_group);
		approach->properties().set("marker_ns", std::string("approach"));
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -1.0;
		approach->setGoal(direction);
		approach->setMinMaxDistance(0.0499, 0.05);
		t.add(std::unique_ptr<Stage>(approach));
	}

	{
		// touch pose generator
		auto touch = new GenerateTouchPose();
		PropertyMap& touch_props = touch->properties();
		touch_props.configureInitFrom(Stage::PARENT, {"object", "eef"});
		touch->setPreGraspPose("touch");
		touch->setAngleDelta(0.2);
		touch->setMonitoredStage(initial);

		auto ik = new ComputeIK("compute ik", std::unique_ptr<Stage>(touch));
		ik->setEndEffector(eef);
		// inherit from parent instead
		// ik->setIKFrame(Eigen::Translation3d(0, 0.05, 0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()), tool_frame);
		PropertyMap& ik_props = ik->properties();
		ik_props.configureInitFrom(Stage::INTERFACE, {"target_pose"});  // derived from child's solution
		touch_props.exposeTo(ik_props, { "object" });
		ik_props.configureInitFrom(Stage::PARENT, {"object", "ik_frame"});
		ik_props.exposeTo(t.properties(), {"object", "ik_frame"});

		t.add(std::unique_ptr<Stage>(ik));
	}

	{  // lift
		auto approach = new stages::MoveRelative("lift", cartesian);
		approach->setGroup(arm_group);
		approach->properties().set("marker_ns", std::string("lift"));
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 1.0;
		approach->setGoal(direction);
		approach->setMinMaxDistance(0.0499, 0.05);
		t.add(std::unique_ptr<Stage>(approach));
	}

	return task;
}

} } }
