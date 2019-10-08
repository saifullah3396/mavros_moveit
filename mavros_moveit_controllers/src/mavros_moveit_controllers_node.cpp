#include <ros/ros.h>
#include <mavros_moveit_controllers/controller_base.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavros_moveit_controller");
  ros::NodeHandle p_nh("~");
  std::string controller_type;
  p_nh.param<std::string>("controller_type", controller_type, "position");
  auto controller = 
    mavros_moveit_controllers::ControllerBase::makeController(controller_type);
  controller->init();
  controller->update();
  delete controller;
  return 0;
}