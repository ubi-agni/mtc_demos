#include <stages/grasping_tasks.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

void spawnObject(moveit::planning_interface::PlanningSceneInterface &psi, int primitive_type,
                 float x_pos, float rot_angle, bool is_lying, geometry_msgs::PoseStamped& pose_msg) {

	double quat[4];  // x y z w
	// rotation about z-axis
	Eigen::Isometry3d pose = Eigen::AngleAxisd(rot_angle/180.0*M_PI, Eigen::Vector3d::UnitZ())
	                         * Eigen::Translation3d::Identity();

	double height = 0.25;
	double radius = 0.03;
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = x_pos;
	o.primitive_poses[0].position.y = 0.30;
	o.primitive_poses[0].position.z = height / 2.0;
	o.primitive_poses[0].orientation.w = 1.0;

	o.primitives.resize(1);
	o.primitives[0].type = primitive_type;
	switch (primitive_type) {
	case shape_msgs::SolidPrimitive::CYLINDER:
		o.primitives[0].dimensions = {height, radius};
		if (is_lying) {
			o.primitive_poses[0].position.z = radius;
			pose = pose * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY());
		}
		break;
	case shape_msgs::SolidPrimitive::BOX:
		o.primitives[0].dimensions = {3*radius, 2*radius, height};
		if (is_lying) {
			o.primitive_poses[0].position.z = o.primitives[0].dimensions[0] / 2.;
			pose = pose * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY());
		}
		break;
	case shape_msgs::SolidPrimitive::SPHERE:
		o.primitives[0].dimensions = {radius};
		o.primitive_poses[0].position.z = radius;
		break;
	}

	o.primitive_poses[0].orientation = tf2::toMsg(Eigen::Quaterniond(pose.rotation()));
	if (!psi.applyCollisionObject(o))
		throw std::runtime_error("Failed to spawn object: " + o.id);

	pose_msg.header.frame_id = o.header.frame_id;
	pose_msg.pose = o.primitive_poses[0];
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pa10");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::PlanningSceneInterface psi;
	geometry_msgs::PoseStamped place_pose;
	spawnObject(psi, shape_msgs::SolidPrimitive::BOX, 0.1, 0.0, false, place_pose);
	place_pose.pose.position.x = -0.3;

	auto task = std::unique_ptr<Task>(bimodalPickPlace(place_pose));
	task->setProperty("object", std::string("object"));

	try {
		if (task->plan())
			task->introspection().publishSolution(*task->solutions().front());
	} catch (const InitStageException &e) {
		std::cerr << "planning failed with exception" << std::endl << e << *task;
	}
	ros::waitForShutdown();
}
