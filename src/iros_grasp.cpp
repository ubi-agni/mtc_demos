#include <stages/grasping_tasks.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <random_numbers/random_numbers.h>
#include "test_utils.h"
#include <iostream>
#include <sstream>
#include <fstream>

#define XMIN	-0.6
#define XMAX	0.6
#define XSTEPS	0.2
#define ROTSTEPS	45
#define ROTMAX	360

using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;
static random_numbers::RandomNumberGenerator rng;

void spawnObject(moveit::planning_interface::PlanningSceneInterface &psi, int primitive_type, float x_pos, float rot_angle, bool is_lying, geometry_msgs::PoseStamped& pose_msg) {
	
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
	unsigned int object_type=1;
	std::stringstream object_type_sstr;
	bool lying = false;
	if (argc >= 2)
	{
		std::cout << "reading object type\n";
		object_type_sstr << argv[1];
		object_type_sstr >> object_type;
		std::cout << "object type " << object_type << std::endl;
		if (!(object_type == 1 || object_type == 2 || object_type == 3 || object_type == 11 || object_type == 33))
		{
			std::cerr << "unsupported object type, use (1=BOX, 2=SPHERE, 3=CYLINDER, 11=LYING BOX, 33=LYING CYLINDER)\n";
			exit(-1);
		}
		if (object_type == 11 || object_type == 33)
		{
			object_type = object_type%10;
			lying  = true;
		}
	}

	std::string csv_out_filename = "";
	if (argc >= 3)
		csv_out_filename = argv[2];

	// prepare output
	std::stringstream header_ss;
	header_ss << "# id,primitive_type,pos_x,rot_angle_around_z,num_sol,";
	header_ss << "0=right,arm_sol, arm_grasps, arm_valid_grasps, arm_places, arm_valid_places,";
	header_ss << "1=left ,arm_sol, arm_grasps, arm_valid_grasps, arm_places, arm_valid_places";
	std::stringstream csv_out;
	std::ofstream csv_out_file;
	if (csv_out_filename != "")
	{
		csv_out_file.open(csv_out_filename);
		csv_out_file << header_ss.str() << "\n";
		if (csv_out_file.bad())
		{
			std::cerr << "error opening file: " << csv_out_filename << std::endl;
			csv_out_file.close();
			exit(-1);
		}
	}

	std::vector<std::string> arms = {"right", "left"};
	// create the psi once to accelerate the process and not destroy the service clients everytime
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::spinOnce();

	ContainerBase::StageCallback stageProcessor = 
			[](const Stage& stage, int depth) -> bool {
		// this method is called for each child stage of a given parent
		std::cout << "stage name: " << stage.name() <<  " at depth " << depth << std::endl;
		return true;
	};

	while (true) {
		// spawn a random object and retrieve it's pose
		/*geometry_msgs::PoseStamped pose;
		spawnObjects(psi, rng.uniformInteger(1, 3), pose); 
		*/
		// spawn each object type, at N different positions, for M different rotation along z
		unsigned int count=0;
		for (unsigned int primitive_type = object_type; primitive_type < object_type+1 && ros::ok(); primitive_type++){
			std::cout << "spawning primitive type " << primitive_type << std::endl;
			for (float x_pos = XMIN; x_pos <= XMAX+0.01 && ros::ok(); x_pos+=XSTEPS){
				int max_rot_angle = 1;
        // no rot for spheres, or for standing cylinders
				//if ((lying && object_type == 3)|| object_type == 1)
						max_rot_angle = ROTMAX;
				for (unsigned int rot_angle = 0; rot_angle < max_rot_angle && ros::ok(); rot_angle+=ROTSTEPS){
					csv_out << count++ << "," << primitive_type << "," << x_pos <<  "," << rot_angle << ",";
					geometry_msgs::PoseStamped pose;
					std::cout <<  "--->   at pos_x " << x_pos << ", rotation angle " << rot_angle << "\n";
					// not lying
					spawnObject(psi, primitive_type, x_pos, rot_angle, lying, pose);

					// place always at same position and orientation
					pose.pose.position.x = 0.0;
					pose.pose.position.y = 0.3;
					pose.pose.position.z = 0.25 / 2.0 + 0.01;
					tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(), pose.pose.orientation);

					auto task = std::unique_ptr<Task>(bimodalPickPlace(pose));
					task->setProperty("object", std::string("object"));
					if (task->plan())
					{
						task->introspection().publishSolution(*task->solutions().front());
					}
					int num_sol = task->numSolutions();
					// extract statistics 
					//std::cout << "num solutions: " << num_sol << std::endl; // main_stage->solutions().size() << std::endl;
					csv_out << num_sol;
					for (auto it=arms.begin(); it != arms.end(); it++)
					{
						csv_out << ",";
						unsigned int alternative_sol=0, alternative_grasps=0, alternative_places=0, alternative_valid_grasps=0, alternative_valid_places=0;
						
						moveit::task_constructor::Stage *alternative = task->stages()->findChild("alternatives/" + *it);
						moveit::task_constructor::Stage *grasp_provider = task->stages()->findChild("alternatives/" + *it + "/pick/grasp/compute ik/grasp_provider");
						moveit::task_constructor::Stage *place_provider = task->stages()->findChild("alternatives/" + *it + "/place/ungrasp/compute ik/place pose");
						moveit::task_constructor::Stage *grasp_ik_valid = task->stages()->findChild("alternatives/" + *it + "/pick/grasp/compute ik");
						moveit::task_constructor::Stage *place_ik_valid = task->stages()->findChild("alternatives/" + *it + "/place/ungrasp/compute ik");

						if(alternative)
						{
							alternative_sol = alternative->solutions().size();
							//std::cout << "num solution for " << *it << " arm: " << alternative_sol << std::endl;
						}
						if(grasp_provider)
						{
							alternative_grasps = grasp_provider->solutions().size();
							//std::cout << "num " << *it << " gen grasps: " << alternative_grasps << std::endl;
						}
						if(grasp_ik_valid)
						{
							alternative_valid_grasps = grasp_ik_valid->solutions().size();
							//std::cout << "num " << *it << " ik valid grasps: " << alternative_valid_grasps << std::endl;
						}
						
						if(place_provider)
						{
							alternative_places = place_provider->solutions().size();
							//std::cout << "num " << *it << " gen  valid grasps: " << alternative_places << std::endl;
						}
						
						if(place_ik_valid)
						{
							alternative_valid_places = place_ik_valid->solutions().size();
							//std::cout << "num " << *it << " ik valid place: " << alternative_valid_places << std::endl;
						}

						if (*it == "left")
							csv_out << "1,";
						else
							csv_out << "0,";
						csv_out << alternative_sol << "," << alternative_grasps << "," << alternative_valid_grasps << 
						                              "," << alternative_places <<  "," << alternative_valid_places;
					}
					csv_out << "\n";
					// write to file
					if (csv_out_file.is_open())
					{
						csv_out_file << csv_out.str();
						csv_out_file.flush();
					}
					else
					{
						std::cout << header_ss.str() << "\n" << csv_out.str();
					}
					csv_out.clear();
					csv_out.str("");
				}
			}
		}
		std::cout << "press q to quit\n" ;
		char ch; std::cin >> ch;
		if (ch == 'q')
			break;
	}
	if (csv_out_file.is_open())
		csv_out_file.close();
}
