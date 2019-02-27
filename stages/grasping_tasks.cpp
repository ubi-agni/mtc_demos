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

#include "grasping_tasks.h"
#include "grasp_provider.h"
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
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

namespace moveit { namespace task_constructor { namespace stages {

Stage* allowCollisions(const std::string& name,
                       const std::vector<std::string>& first,
                       const std::vector<std::string>& second,
                       bool allow = true)
{
	auto allow_touch = new ModifyPlanningScene(name);
	allow_touch->allowCollisions(first, second, allow);
	return allow_touch;
}

Task* initAndFixCollisions(Stage** initial_out) {
	Task* task = new Task();
	Task& t = *task;

	Stage* initial;
	t.add(Stage::pointer(initial = new stages::CurrentState("current")));
	t.add(Stage::pointer(initial =
	      allowCollisions("allow fingertip-table collision", {"frame"},
	{"rh_ffdistal", "rh_mfdistal", "rh_rfdistal", "rh_lfdistal", "rh_thdistal",
	 "lh_ffdistal", "lh_mfdistal", "lh_rfdistal", "lh_lfdistal", "lh_thdistal"})));

	auto fix = new stages::FixCollisionObjects();
	fix->setMaxPenetration(0.02);
	geometry_msgs::Vector3 correction;
	correction.z = 0.002;
	fix->setDirection(correction);
	t.add(Stage::pointer(fix));
	initial = fix;

	if (initial_out)
		*initial_out = initial;
	return task;
}

// add a pick sub task to container and return the created pick container
ContainerBase* addPick(ContainerBase& container, Stage* initial,
                       const std::string& side, const std::string& name="pick") {
	const std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	const std::string eef = side.substr(0,1) + "a_tool_mount";
	const std::string arm = side + "_arm";
	const std::string hand = side + "_hand";
	const std::string grasp_config = "shadow_" + side + "_handed_limited";

	// planner used for connect
	auto interpolate = std::make_shared<solvers::JointInterpolationPlanner>();
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{hand, interpolate}, {arm, pipeline}};
	auto connect = new stages::Connect("approach pick", planners);
	container.insert(Stage::pointer(connect));

	// grasp generator
	auto grasp_generator = new stages::GraspProvider();
	grasp_generator->setProperty("config", grasp_config);
	grasp_generator->setMonitoredStage(initial);

	auto grasp = new stages::SimpleGrasp(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	grasp->setIKFrame(tool_frame);

	// pick container, using the generated grasp generator
	auto pick = new stages::Pick(Stage::pointer(grasp), name);
	PropertyMap& props = pick->properties();
	props.set("eef", eef);
	props.declare<std::string>("object");
	props.configureInitFrom(Stage::PARENT, { "object" });

	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.z = 1.0;
	pick->setApproachMotion(approach, 0.05, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "frame";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	container.insert(Stage::pointer(pick));
	return pick;
}

// add a pick sub task to container and return the created pick container
ContainerBase* addPlace(ContainerBase& container, Stage* grasped, const std::string& side,
                        const geometry_msgs::PoseStamped& p, const std::string& name="place") {
	const std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	const std::string eef = side.substr(0,1) + "a_tool_mount";
	const std::string arm = side + "_arm";
	const std::string hand = side + "_hand";
	const std::string grasp_config = "shadow_" + side + "_handed_limited";

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{arm, pipeline}};
	auto connect = new stages::Connect("approach " + name, planners);
	container.insert(Stage::pointer(connect));

	// place generator
	auto place_generator = new GeneratePlacePose();
	place_generator->setPose(p);
	place_generator->properties().configureInitFrom(Stage::PARENT);
	place_generator->setMonitoredStage(grasped);
	place_generator->setForwardedProperties({"pregrasp", "grasp"});

	auto ungrasp = new stages::SimpleUnGrasp(std::unique_ptr<MonitoringGenerator>(place_generator));
	ungrasp->setIKFrame(tool_frame);

	// pick container, using the generated grasp generator
	auto place = new stages::Place(Stage::pointer(ungrasp), name);
	PropertyMap& props = place->properties();
	props.set("eef", eef);
	props.declare<std::string>("object");
	props.configureInitFrom(Stage::PARENT, { "object" });

	geometry_msgs::TwistStamped retract;
	retract.header.frame_id = tool_frame;
	retract.twist.linear.z = -1.0;
	place->setRetractMotion(retract, 0.05, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "frame";
	lift.twist.linear.z = -1.0;
	place->setPlaceMotion(lift, 0.01, 0.1);

	container.insert(Stage::pointer(place));
	return place;
}

Task* pick(const std::string& name, const std::string& side) {
	Stage* initial;
	Task* task = initAndFixCollisions(&initial);
	task->stages()->setName(name);
	addPick(*task->stages(), initial, side);
	return task;
}

Task* pickPlace(const geometry_msgs::PoseStamped& object_target_pose,
                const std::string& name, const std::string& side) {
	Task* task = pick(name, side);
	addPlace(*task->stages(), task->stages()->findChild("pick/grasp"), side, object_target_pose);
	return task;
}

Task* bimodalTask(const std::string& name,
                  const std::function<void(ContainerBase&, Stage*, const std::string&)>& f)
{
	Stage* initial;
	Task* task = initAndFixCollisions(&initial);
	task->stages()->setName(name);

	auto alternatives = std::make_unique<Alternatives>();
	PropertyMap& props = alternatives->properties();
	props.declare<std::string>("object");
	props.configureInitFrom(Stage::PARENT, { "object" });

	for (const std::string& side : {"left", "right"})
	{
		auto container = new SerialContainer(side);
		PropertyMap& props = container->properties();
		props.declare<std::string>("object");
		props.configureInitFrom(Stage::PARENT, { "object" });

		f(*container, initial, side);
		alternatives->insert(Stage::pointer(container));
	}

	task->add(std::move(alternatives));
	return task;
}

Task* bimodalPick(const std::string& name)
{
	return bimodalTask(name, [](ContainerBase& container, Stage* initial, const std::string& side) {
		addPick(container, initial, side);
	});
}

Task* bimodalPickPlace(const geometry_msgs::PoseStamped& object_target_pose, const std::string& name)
{
	return bimodalTask(name, [&](ContainerBase& container, Stage* initial, const std::string& side) {
		addPick(container, initial, side);
		addPlace(container, container.findChild("pick/grasp"), side, object_target_pose);
	});
}

Task* bimanualPickPlace(const geometry_msgs::PoseStamped& object_target_pose,
                        const std::string& name)
{
	// planners
	auto interpolate = std::make_shared<solvers::JointInterpolationPlanner>();
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTkConfigDefault");

	Task* task = pick(name, "right");
	Stage* referenced_stage;
	{
		auto handover = new stages::MoveTo("move to handover", cartesian);
		handover->setProperty("group", "right_arm");
		handover->setTimeout(10.0);

		geometry_msgs::PoseStamped target;
		target.header.frame_id = "world";
		target.pose.position.x =  0.0;
		target.pose.position.y =  0.3;
		target.pose.position.z =  0.2;
		target.pose.orientation.x = 0.70711;
		target.pose.orientation.x = 0.0;
		target.pose.orientation.z = 0.70711;
		target.pose.orientation.w = 0.0;
		handover->setGoal(target);
		handover->setIKFrame("rh_tool_frame");
		referenced_stage = handover;
		task->add(Stage::pointer(handover));
	}
#if 0
	// connect to pick left
	stages::Connect::GroupPlannerVector planners = {{"left_hand", interpolate}, {"left_arm", pipeline}};
	auto connect = new stages::Connect("approach pick left", planners);
	task->add(Stage::pointer(connect));
#endif
	ContainerBase* pick = addPick(*task->stages(), referenced_stage, "left", "pick left");
	pick->remove(pick->findChild("lift object"));

	ContainerBase* grasp = static_cast<ContainerBase*>(pick->findChild("grasp"));
	auto* ungrasp = new stages::SimpleUnGrasp();
	ungrasp->setEndEffector("right_hand");
	ungrasp->properties().configureInitFrom(Stage::PARENT, {"object"});
	ungrasp->setProperty("pregrasp", "open");

	// HACK: restrict dir, container "ungrasp" should have its dir restricted
	static_cast<PropagatingEitherWay*>(ungrasp->findChild("open gripper"))->restrictDirection(PropagatingEitherWay::FORWARD);
	grasp->insert(Stage::pointer(ungrasp), -2);

	addPlace(*task->stages(), pick->findChild("grasp"), "left", object_target_pose);

	return task;
}

} } }
