cmake_minimum_required(VERSION 2.8.3)
project(moveit_external_octomap_updater)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-std=c++11)

find_package(catkin COMPONENTS
  moveit_core
  moveit_ros_perception
  pluginlib
  roscpp
  tf
  REQUIRED
  octomap_ros)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(include)

add_library(moveit_external_octomap_updater src/moveit_external_octomap_updater.cpp)
target_link_libraries(moveit_external_octomap_updater ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_external_octomap_updater LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_external_octomap_updater_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
