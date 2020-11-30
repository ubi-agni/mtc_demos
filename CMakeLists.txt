cmake_minimum_required(VERSION 3.1.3)
project(mtc_demos)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
	moveit_ros_planning_interface
	moveit_task_constructor_core
	rviz_marker_tools
	tf2_eigen
	grasping_msgs
	rostest
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mtc_demos
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(stages)
add_subdirectory(src)
add_subdirectory(python)
add_subdirectory(test)
