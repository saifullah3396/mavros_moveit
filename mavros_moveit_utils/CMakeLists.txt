cmake_minimum_required(VERSION 2.8.3)
project(mavros_moveit_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	tf
	geometry_msgs)
	
catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp tf geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
FILE (GLOB SOURCE src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})