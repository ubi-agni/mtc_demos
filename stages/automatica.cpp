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
#include <moveit/task_constructor/stages/fix_collision_objects.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include "grasp_provider.h"
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
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

Task* grasp(const std::string& name, const std::string& side)
{
	Task* task = new Task(name);
	Task& t = *task;

	Stage* initial = new stages::CurrentState("current");
	t.add(std::unique_ptr<Stage>(initial));

	{
		auto fix = new stages::FixCollisionObjects();
		fix->setMaxPenetration(0.04);
		t.add(std::unique_ptr<Stage>(fix));
		initial = fix;
	}


	std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	std::string eef = side.substr(0,1) + "a_tool_mount";
	std::string arm = side + "_arm";
	std::string config = "shadow_" + side + "_handed_limited";

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	pipeline->properties().set("max_velocity_scaling_factor", 0.1);
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{side + "_hand", pipeline}, {arm, pipeline}};
	auto connect = new stages::Connect("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	connect->properties().set("merge_mode", stages::Connect::SEQUENTIAL);
	t.add(std::unique_ptr<Stage>(connect));

	// grasp generator
	auto grasp_generator = new stages::GraspProvider();
	grasp_generator->setProperty("config", config);
	grasp_generator->setMonitoredStage(initial);

	auto grasp = new stages::SimpleGrasp(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	grasp->setIKFrame(tool_frame);

	// pick container, using the generated grasp generator
	auto pick = new stages::Pick(std::unique_ptr<Stage>(grasp), side);
	pick->setProperty("eef", eef);
	pick->properties().configureInitFrom(Stage::PARENT, { "object" });
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.z = 1.0;
	pick->setApproachMotion(approach, 0.05, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "frame";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	// finalize
	t.add(std::unique_ptr<Stage>(pick));

	return task;
}

Task* graspAndDrop(const std::string& name, const std::string& side)
{
	Task* task = grasp(name, side);
	Task& t = *task;

	std::string arm = side + "_arm";
	std::string hand = side + "_hand";
	std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	std::string eef = side.substr(0,1) + "a_tool_mount";

	// planner used for connect
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->properties().set("max_velocity_scaling_factor", 0.1);
	auto interpolate = std::make_shared<solvers::JointInterpolationPlanner>();

	{
		auto move = new stages::MoveRelative("turn", cartesian);
		move->setGroup(arm);
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = tool_frame;
		twist.twist.angular.y = side == "left" ? -0.2 : 0.2;
		move->setGoal(twist);
		move->setProperty("marker_ns", "turn");
		t.add(std::unique_ptr<Stage>(move));
	}

	{
		auto detach = new ModifyPlanningScene("detach object");
		PropertyMap& p = detach->properties();
		p.set("eef", eef);
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT, { "object" });

		detach->setCallback([](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
				const std::string& eef = p.get<std::string>("eef");
				moveit_msgs::AttachedCollisionObject obj;
				obj.object.operation = (int8_t) moveit_msgs::CollisionObject::REMOVE;
				obj.link_name = scene->getRobotModel()->getEndEffector(eef)->getEndEffectorParentGroup().second;
				obj.object.id = p.get<std::string>("object");
				scene->processAttachedCollisionObjectMsg(obj);
			});
		t.add(std::unique_ptr<Stage>(detach));
	}

	{
		auto move = new stages::MoveTo("open", interpolate);
		move->setGroup(hand);
		move->setGoal("open");
		t.add(std::unique_ptr<Stage>(move));
	}

	{
		auto move = new stages::MoveRelative("side", cartesian);
		move->setGroup(arm);
		move->setProperty("marker_ns", "side");
		geometry_msgs::Vector3Stamped v;
		v.header.frame_id = "world";
		v.vector.x = side == "left" ? 0.15 : -0.15;
		move->setGoal(v);
		t.add(std::unique_ptr<Stage>(move));
	}

	return task;
}

Task* juggleStart()
{
	const std::string side = "left";
	Task* task = grasp("", side);
	Task& t = *task;

	{
		auto detach = new ModifyPlanningScene("detach object");
		PropertyMap& p = detach->properties();
		p.set("eef", side.substr(0,1) + "a_tool_mount");
		p.declare<std::string>("object");
		p.configureInitFrom(Stage::PARENT, { "object" });

		detach->setCallback([](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p){
				const std::string& eef = p.get<std::string>("eef");
				moveit_msgs::AttachedCollisionObject obj;
				obj.object.operation = (int8_t) moveit_msgs::CollisionObject::REMOVE;
				obj.link_name = scene->getRobotModel()->getEndEffector(eef)->getEndEffectorParentGroup().second;
				obj.object.id = p.get<std::string>("object");
				scene->processAttachedCollisionObjectMsg(obj);
			});
		t.add(std::unique_ptr<Stage>(detach));
	}

	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	pipeline->properties().set("max_velocity_scaling_factor", 0.1);

	for (const std::string& side: {"left", "right"}) {
		auto move = new stages::MoveTo(side + " arm", pipeline);
		move->restrictDirection(stages::MoveRelative::FORWARD);
		move->setGroup(side + "_arm");
		move->setGoal("juggleLeftA");
		t.add(std::unique_ptr<Stage>(move));
	}

	for (const std::string& side: {"left", "right"}) {
		auto interpolate = std::make_shared<solvers::JointInterpolationPlanner>();
		auto move = new stages::MoveTo(side + " hand", interpolate);
		move->setGroup(side + "_hand");
		move->setGoal("juggle");
		t.add(std::unique_ptr<Stage>(move));
	}

	return task;
}

Task* juggle()
{
	Task* task = juggleStart();
	Task& t = *task;

	// planners
	auto planner = std::make_shared<solvers::JointInterpolationPlanner>();
	planner->properties().set("max_velocity_scaling_factor", 0.1);

	auto move = new stages::MoveTo("turn left arm", planner);
	move->restrictDirection(stages::MoveRelative::FORWARD);
	move->setGroup("left_arm");
	move->setGoal("juggleLeftB");
	t.add(std::unique_ptr<Stage>(move));

	for (const std::string& side: {"left", "right"}) {
		move = new stages::MoveTo(side, planner);
		move->restrictDirection(stages::MoveRelative::FORWARD);
		move->setGroup(side + "_arm");
		move->setGoal("juggleMiddleB");
		t.add(std::unique_ptr<Stage>(move));
	}

	for (const std::string& side: {"left", "right"}) {
		auto move = new stages::MoveTo(side, planner);
		move->restrictDirection(stages::MoveRelative::FORWARD);
		move->setGroup(side + "_arm");
		move->setGoal("juggleRightA");
		t.add(std::unique_ptr<Stage>(move));
	}

	move = new stages::MoveTo("turn right arm", planner);
	move->restrictDirection(stages::MoveRelative::FORWARD);
	move->setGroup("right_arm");
	move->setGoal("juggleRightB");
	t.add(std::unique_ptr<Stage>(move));

	for (const std::string& side: {"right", "left"}) {
		auto move = new stages::MoveTo(side, planner);
		move->restrictDirection(stages::MoveRelative::FORWARD);
		move->setGroup(side + "_arm");
		move->setGoal("juggleMiddleB");
		t.add(std::unique_ptr<Stage>(move));
	}

	for (const std::string& side: {"right", "left"}) {
		auto move = new stages::MoveTo(side + " arm", planner);
		move->restrictDirection(stages::MoveRelative::FORWARD);
		move->setGroup(side + "_arm");
		move->setGoal("juggleLeftA");
		t.add(std::unique_ptr<Stage>(move));
	}

	return task;
}

} } }
