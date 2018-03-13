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

add_executable(baxter src/baxter.cpp)
target_link_libraries(baxter ${catkin_LIBRARIES})

add_executable(pr2 src/pr2.cpp)
target_link_libraries(pr2 ${catkin_LIBRARIES})

add_executable(pa10 src/pa10.cpp)
target_link_libraries(pa10 ${catkin_LIBRARIES})

add_executable(pepper src/pepper.cpp)
target_link_libraries(pepper ${catkin_LIBRARIES})

add_executable(yumi src/yumi.cpp)
target_link_libraries(yumi ${catkin_LIBRARIES})


add_executable(bimodal_baxter src/bimodal_baxter.cpp)
target_link_libraries(bimodal_baxter ${catkin_LIBRARIES})

add_executable(bimodal_pr2 src/bimodal_pr2.cpp)
target_link_libraries(bimodal_pr2 ${catkin_LIBRARIES})

add_executable(bimodal_pa10 src/bimodal_pa10.cpp)
target_link_libraries(bimodal_pa10 ${catkin_LIBRARIES})

add_executable(bimodal_pepper src/bimodal_pepper.cpp)
target_link_libraries(bimodal_pepper ${catkin_LIBRARIES})
