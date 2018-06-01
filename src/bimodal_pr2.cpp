#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace moveit::task_constructor;

void spawnObject(bool right) {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "base_link";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.53;
	o.primitive_poses[0].position.y = 0.05;
	o.primitive_poses[0].position.z = 0.84;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

void fill(ParallelContainerBase &container, Stage* initial_stage, bool right_side) {
	std::string side = right_side ? "right" : "left";
	std::string tool_frame = side.substr(0,1) + "_gripper_tool_frame";
	std::string eef = side + "_gripper";
	std::string arm = side + "_arm";

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setTimeout(8.0);
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);

	// grasp generator
	auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
	grasp_generator->setAngleDelta(.2);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
	grasp->setIKFrame(tool_frame);
	grasp->setPreGraspPose("open");
	grasp->setGraspPose("closed");
	grasp->setMonitoredStage(initial_stage);

	// pick container, using the generated grasp generator
	auto pick = std::make_unique<stages::Pick>(std::move(grasp), side);
	pick->setProperty("eef", eef);
	pick->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.x = 1.0;
	pick->setApproachMotion(approach, 0.03, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "base_link";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	pick->insert(std::move(connect), 0);
	container.insert(std::move(pick));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "bimodal");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	Task t;

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>();
	initial_stage = initial.get();
	t.add(std::move(initial));

	auto parallel = std::make_unique<Alternatives>();
	fill(*parallel, initial_stage, true);
	fill(*parallel, initial_stage, false);

	t.add(std::move(parallel));

	try {
		char ch;
		spawnObject(true);
		t.plan();
		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;

		spawnObject(false);
		t.plan();
		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		std::cerr << t;
		return EINVAL;
	}

	return 0;
}
