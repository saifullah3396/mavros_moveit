cmake_minimum_required(VERSION 2.8.3)
project(mavros_moveit_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	mavros_msgs 
	control_msgs
	control_toolbox
	mavros_moveit_utils)
	
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs control_msgs control_toolbox mavros_moveit_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})
FILE (GLOB SOURCE src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})