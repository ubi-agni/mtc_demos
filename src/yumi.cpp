#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace moveit::task_constructor;

void spawnObject() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "yumi_body";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.25;
	o.primitive_poses[0].position.y = 0.18;
	o.primitive_poses[0].position.z = 0.1;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.2;
	o.primitives[0].dimensions[1] = 0.02;
	psi.applyCollisionObject(o);

	o.id = "table";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.44;
	o.primitive_poses[0].position.y = 0.0;
	o.primitive_poses[0].position.z = 0.07;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0] = 0.6;
	o.primitives[0].dimensions[1] = 1.0;
	o.primitives[0].dimensions[2] = 0.06;
	psi.applyCollisionObject(o);
}

void createTask(Task& t) {
	spawnObject();
	std::string tool_frame = "yumi_link_7_r";
	std::string eef = "right_hand";
	std::string arm = "right_arm";

	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setTimeout(8.0);
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto cartesian = std::make_shared<solvers::CartesianPath>();

	Stage* referenced_stage = nullptr;
	{
		// fetch initial state from move_group
		auto initial = std::make_unique<stages::CurrentState>("current state");
		referenced_stage = initial.get();
		t.add(std::move(initial));

		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));

		// grasp generator
		auto grasp_generator = std::make_unique<stages::SimpleGrasp>();
		grasp_generator->setToolToGraspTF(Eigen::Translation3d(0.0, -0.05, 0.13) *
		                                  Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()),
		                                  tool_frame);

		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("closed");
		grasp_generator->setMonitoredStage(referenced_stage);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp_generator), "pick with right");
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = tool_frame;
		approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "world";
		lift.twist.linear.z = 1.0;
		pick->setLiftMotion(lift, 0.03, 0.05);
		t.add(std::move(pick));
	}
	{
		auto handover = std::make_unique<stages::MoveTo>("move to handover", cartesian);
		handover->setProperty("group", arm);

		// TODO: specify that attached object should move to a specific location
		geometry_msgs::PointStamped target;
		target.header.frame_id = "world";
		target.point.x =  0.40;
		target.point.y = -0.12;
		target.point.z =  0.2;
		handover->setGoal(target);
		referenced_stage = handover.get();
		t.add(std::move(handover));
	}

	/************************************************************************************/
	tool_frame = "yumi_link_7_l";
	eef = "left_hand";
	arm = "left_arm";
	{
		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));

		// grasp generator
		auto grasp_generator = std::make_unique<stages::SimpleGrasp>();
		grasp_generator->setToolToGraspTF(Eigen::Translation3d(0.0, 0.05, 0.13) *
		                                  Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()),
		                                  tool_frame);

		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("closed");
		grasp_generator->setMonitoredStage(referenced_stage);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp_generator), "pick with left");
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = tool_frame;
		approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "world";
		lift.twist.linear.z = 0.0;
		pick->setLiftMotion(lift, 0.0, 0.0);
		t.add(std::move(pick));
	}
	{
		auto place = std::make_unique<stages::MoveTo>("move to place", cartesian);
		place->setProperty("group", arm);

		// TODO: specify that attached object should move to a specific location
		geometry_msgs::PointStamped target;
		target.header.frame_id = "world";
		target.point.x = 0.4;
		target.point.y = 0.5;
		target.point.z = 0.1;
		place->setGoal(target);
		t.add(std::move(place));
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "yumi");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;
	createTask(t);
	try {
		t.plan();
		std::cout << "waiting for any key + <enter>\n";
		char ch;
		std::cin >> ch;
	}
	catch (const InitStageException &e) {
		std::cerr << e << t;
		return EINVAL;
	}

	return 0;
}
