#include <mavros_moveit_controllers/controller_base.h>
#include <mavros_moveit_controllers/position_controller.h>
#include <mavros_moveit_utils/utils.h>

namespace mavros_moveit_controllers {

PositionController::PositionController() {}
PositionController::~PositionController() {}

void PositionController::init()
{
    // Initialize base parameters
    ControllerBase::init();
    target_.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_VX | 
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_AFX | 
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ;

}

void PositionController::commandCb(const geometry_msgs::PoseStamped::ConstPtr& cmd_pose) {
    if (!state_received_) {
        ROS_WARN("Controller cannot generate command due to unknown mavros state.");
        return;
    }

    if (!pose_received_) {
        ROS_WARN("Controller cannot generate command due to unknown robot pose.");
        return;
    }
    target_.header.stamp = ros::Time::now();
    target_.position = cmd_pose->pose.position;
    target_.yaw = mavros_moveit_utils::getYaw(cmd_pose->pose.orientation);
    local_raw_pub_.publish(target_);
    last_update_time_ = ros::Time::now();
}

}