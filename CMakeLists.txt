cmake_minimum_required(VERSION 3.10.2)
project(mtc_demos)

find_package(catkin REQUIRED COMPONENTS
	moveit_ros_planning_interface
	moveit_task_constructor_core
	rviz_marker_tools
	eigen_conversions
)

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
add_subdirectory(test)
