cmake_minimum_required(VERSION 2.8.3)
project(mavros_moveit_controllers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	message_generation
	geometry_msgs 
	mavros_msgs 
	control_msgs
	control_toolbox
	mavros_moveit_utils)
	
add_service_files(
  FILES
  SetOffboard.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs control_msgs control_toolbox mavros_moveit_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})
FILE (GLOB SOURCE src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCE})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})