cmake_minimum_required(VERSION 2.8.3)
project(mavros_moveit_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	mavros_msgs 
	control_msgs
	control_toolbox)
	
catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs control_msgs control_toolbox
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
FILE (GLOB SOURCE src/*.cpp)
add_library(mavros_moveit_controllers ${SOURCE})
target_link_libraries(mavros_moveit_controllers ${catkin_LIBRARIES})