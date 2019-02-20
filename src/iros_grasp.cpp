#include <stages/grasping_tasks.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "test_utils.h"

using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

void spawnObjects(int primitive, geometry_msgs::PoseStamped& pose) {
	moveit::planning_interface::PlanningSceneInterface psi;

	double height = 0.25;
	double radius = 0.03;
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = -0.3;
	o.primitive_poses[0].position.y = 0.30;
	o.primitive_poses[0].position.z = height / 2.0 + 0.001;
	o.primitive_poses[0].orientation.w = 1.0;

	o.primitives.resize(1);
	o.primitives[0].type = primitive;
	switch (primitive) {
	case shape_msgs::SolidPrimitive::CYLINDER:
		o.primitives[0].dimensions = {height, radius};
		break;
	case shape_msgs::SolidPrimitive::BOX:
		o.primitives[0].dimensions = {4*radius, 3*radius, height};
		break;
	case shape_msgs::SolidPrimitive::SPHERE:
		o.primitives[0].dimensions = {radius};
		o.primitive_poses[0].position.z = radius;
		break;
	}
	psi.applyCollisionObject(o);

	pose.header.frame_id = o.header.frame_id;
	pose.pose = o.primitive_poses[0];
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pa10");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	geometry_msgs::PoseStamped pose;
	spawnObjects(shape_msgs::SolidPrimitive::BOX, pose);
	pose.pose.position.x = 0.3;

	auto task = std::unique_ptr<Task>(bimodalPickPlace(pose));
	task->setProperty("object", std::string("object"));
	task->plan();
	waitForKey();
}
