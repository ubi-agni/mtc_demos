#include <stages/grasping_tasks.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <random_numbers/random_numbers.h>
#include "test_utils.h"

using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;
static random_numbers::RandomNumberGenerator rng;

void spawnObject(moveit::planning_interface::PlanningSceneInterface &psi, int primitive_type, float x_pos, float rot_angle, bool is_lying, geometry_msgs::PoseStamped& pose_msg) {
	
	double quat[4];  // x y z w
	// rotation about z-axis
	Eigen::Isometry3d pose = Eigen::AngleAxisd(rot_angle, Eigen::Vector3d::UnitZ())
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
		rng.quaternion(quat);  // random orientation
		pose = Eigen::Quaterniond(quat);
		break;
	}
	tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation()), o.primitive_poses[0].orientation);
	psi.applyCollisionObject(o);

	pose_msg.header.frame_id = o.header.frame_id;
	pose_msg.pose = o.primitive_poses[0];
}

void spawnObjects(moveit::planning_interface::PlanningSceneInterface &psi, int primitive, geometry_msgs::PoseStamped& pose_msg) {
	//moveit::planning_interface::PlanningSceneInterface psi;
	double quat[4];  // x y z w

	// random rotation about z-axis
	Eigen::Isometry3d pose = Eigen::AngleAxisd(rng.uniformReal(-M_PI/2., M_PI/2.), Eigen::Vector3d::UnitZ())
	                         * Eigen::Translation3d::Identity();

	double height = 0.25;
	double radius = 0.03;
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = -0.3;
	o.primitive_poses[0].position.y = 0.30;
	o.primitive_poses[0].position.z = height / 2.0;
	o.primitive_poses[0].orientation.w = 1.0;

	o.primitives.resize(1);
	o.primitives[0].type = primitive;
	switch (primitive) {
	case shape_msgs::SolidPrimitive::CYLINDER:
		o.primitives[0].dimensions = {height, radius};
		if (int lying = rng.uniformInteger(0, 1)) {
			o.primitive_poses[0].position.z = radius;
			pose = pose * Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY());
		}
		break;
	case shape_msgs::SolidPrimitive::BOX:
		o.primitives[0].dimensions = {3*radius, 2*radius, height};
		switch (rng.uniformInteger(0, 2)) {
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
	case shape_msgs::SolidPrimitive::SPHERE:
		o.primitives[0].dimensions = {radius};
		o.primitive_poses[0].position.z = radius;
		rng.quaternion(quat);  // random orientation
		pose = Eigen::Quaterniond(quat);
		break;
	}
	tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation()), o.primitive_poses[0].orientation);
	psi.applyCollisionObject(o);

	pose_msg.header.frame_id = o.header.frame_id;
	pose_msg.pose = o.primitive_poses[0];
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pa10");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// create the psi once to accelerate the process and not destroy the service clients everytime
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::spinOnce();
	while (true) {
		// spawn a random object and retrieve it's pose
		/*geometry_msgs::PoseStamped pose;
		spawnObjects(psi, rng.uniformInteger(1, 3), pose); 
		*/
		for (unsigned int primitive_type = 1; primitive_type < 4; primitive_type++){
			for (float x_pos = -0.8; x_pos <= 0.8; x_pos+=0.1){
				for (unsigned int rot_angle = 0; rot_angle < 360; rot_angle+=45){
					geometry_msgs::PoseStamped pose;
					std::cout << "spawning primitive type " << primitive_type << " at pos_x " << x_pos << ", rotation angle " << rot_angle << "\n";
					// not lying
					spawnObject(psi, primitive_type, x_pos, rot_angle, false, pose);
					ros::spinOnce();
					ros::Duration(0.5).sleep();
					std::cout << "spawned object\n";

					// place always at same position and orientation
					pose.pose.position.x = 0.0;
					pose.pose.position.y = 0.3;
					pose.pose.position.z = 0.25 / 2.0 + 0.01;
					tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(), pose.pose.orientation);

					auto task = std::unique_ptr<Task>(bimodalPickPlace(pose));
					task->setProperty("object", std::string("object"));
					if (task->plan())
						task->introspection().publishSolution(*task->solutions().front());
					
					char ch; std::cin >> ch;
					if (ch == 'q')
						exit(0);
					
				}
			}
		}
	}
}
