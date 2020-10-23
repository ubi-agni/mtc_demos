#include <stages/grasping_tasks.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.h>
#include <random_numbers/random_numbers.h>
#include "test_utils.h"

using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;
static random_numbers::RandomNumberGenerator rng;

void spawnObjects(int primitive, geometry_msgs::PoseStamped& pose_msg) {
	moveit::planning_interface::PlanningSceneInterface psi;

	// random rotation about z-axis
	Eigen::Isometry3d pose = Eigen::AngleAxisd(20./180.*M_PI, Eigen::Vector3d::UnitZ())
	                         * Eigen::Translation3d::Identity();

	double height = 0.26;
	double radius = 0.076 / 2.;
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = -0.5;
	o.primitive_poses[0].position.y = 0.30;
	o.primitive_poses[0].position.z = height / 2.0;
	o.primitive_poses[0].orientation.w = 1.0;

	o.primitives.resize(1);
	o.primitives[0].type = primitive;
	switch (primitive) {
	case shape_msgs::SolidPrimitive::CYLINDER:
		o.primitives[0].dimensions = {height, radius};
		if (int lying = 1) { // rng.uniformInteger(0, 1)) {
			(void)lying;
			o.primitive_poses[0].position.z = radius;
			pose = pose * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY());
		}
		break;
	case shape_msgs::SolidPrimitive::SPHERE:  // we cannot handover a sphere
	case shape_msgs::SolidPrimitive::BOX:
		o.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		o.primitives[0].dimensions = {2*radius, 2*radius, height};  // milk
#if 0 // lying?
		switch(rng.uniformInteger(0, 2)) {
#else
		switch (0) {
#endif
		case 1:  // rotate about y axis
			o.primitive_poses[0].position.z = o.primitives[0].dimensions[0] / 2.;
			pose = pose * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY());
			break;
		case 2:  // rotate about x axis
			o.primitive_poses[0].position.z = o.primitives[0].dimensions[1] / 2.;
			pose = pose * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitX());
			break;
		default:
			break;
		}
		break;
	}
	o.primitive_poses[0].orientation = tf2::toMsg(Eigen::Quaterniond(pose.rotation()));
	psi.applyCollisionObject(o);

	pose_msg.header.frame_id = o.header.frame_id;
	pose_msg.pose = o.primitive_poses[0];
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pa10");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while (true) {
		// spawn a random object and retrieve it's pose
		geometry_msgs::PoseStamped pose;
		spawnObjects(shape_msgs::SolidPrimitive::CYLINDER, pose);

		// place always at same position and orientation
		pose.pose.position.x = 0.7;
		pose.pose.position.y = 0.3;
		pose.pose.position.z = 0.25 / 2.0 + 0.01;
		pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond::Identity());

		auto task = std::unique_ptr<Task>(bimanualPickPlace(pose));
		task->setProperty("object", std::string("object"));
		try {
			if (task->plan(5))
				task->introspection().publishSolution(*task->solutions().front());
		} catch (const InitStageException &e) {
			std::cerr << e << *task;
		}

		char ch; std::cin >> ch;
		if (ch == 'q') break;
	}
}
