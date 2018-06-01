cmake_minimum_required(VERSION 2.8.3)
project(mtc_demos)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
	add_compile_options(-std=c++14)
else ()
	set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(catkin REQUIRED COMPONENTS
	moveit_ros_planning_interface
	moveit_task_constructor_core
	rviz_marker_tools
	rostest
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

add_executable(baxter src/baxter.cpp)
target_link_libraries(baxter ${catkin_LIBRARIES})

add_executable(pr2 src/pr2.cpp)
target_link_libraries(pr2 ${catkin_LIBRARIES})

add_executable(pa10 src/pa10.cpp)
target_link_libraries(pa10 ${catkin_LIBRARIES})

add_executable(pepper src/pepper.cpp)
target_link_libraries(pepper ${catkin_LIBRARIES})


add_executable(baxter_bimodal src/baxter_bimodal.cpp)
target_link_libraries(baxter_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pr2_bimodal src/pr2_bimodal.cpp)
target_link_libraries(pr2_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pa10_bimodal src/pa10_bimodal.cpp)
target_link_libraries(pa10_bimodal ${catkin_LIBRARIES} gtest)

add_executable(pepper_bimodal src/pepper_bimodal.cpp)
target_link_libraries(pepper_bimodal ${catkin_LIBRARIES} gtest)


add_executable(pepper_bimanual src/pepper_bimanual.cpp)
target_link_libraries(pepper_bimanual ${PROJECT_NAME}_stages ${catkin_LIBRARIES} gtest)


add_executable(yumi_handover src/yumi_handover.cpp)
target_link_libraries(yumi_handover ${catkin_LIBRARIES} gtest)

add_executable(yumi_bimanual src/yumi_bimanual.cpp)
target_link_libraries(yumi_bimanual ${PROJECT_NAME}_stages ${catkin_LIBRARIES} gtest)

add_subdirectory(test)
