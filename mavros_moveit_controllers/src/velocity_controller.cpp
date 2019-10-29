#include <mavros_moveit_controllers/controller_base.h>
#include <mavros_moveit_controllers/velocity_controller.h>
#include <mavros_moveit_utils/utils.h>

namespace mavros_moveit_controllers {

VelocityController::VelocityController()
{
    controllers_.resize(static_cast<int>(CI::count));
}

VelocityController::~VelocityController() {}

void VelocityController::init()
{
    // Initialize base parameters
    ControllerBase::init();

    ros::NodeHandle p_nh("~");
    // Setup of the PID controllers
    setupController(CI::x, "x");
    setupController(CI::y, "y");
    setupController(CI::z, "z");
    setupController(CI::yaw, "yaw");

    target_.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_PX | 
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_AFX | 
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ;

}

void VelocityController::generateCommand(const geometry_msgs::PoseStamped& cmd_pose) {
    target_.velocity.x = 
        computeEffort(
            CI::x, 
            cmd_pose.pose.position.x - current_pose_.pose.position.x, 
            last_update_time_);
    target_.velocity.y = 
        computeEffort(
            CI::y, 
            cmd_pose.pose.position.y - current_pose_.pose.position.y, 
            last_update_time_);
    target_.velocity.z = 
        computeEffort(
            CI::z, 
            cmd_pose.pose.position.z - current_pose_.pose.position.z, 
            last_update_time_);
    tf::Pose tf_curr, tf_cmd;
    tf::poseMsgToTF(current_pose_.pose, tf_curr);
    tf::poseMsgToTF(cmd_pose.pose, tf_cmd);
    auto diff_t = tf_curr.inverseTimes(tf_cmd);
    auto diff_yaw = mavros_moveit_utils::getYaw(diff_t.getRotation());
    target_.yaw_rate =
        computeEffort(
            CI::yaw, 
            diff_yaw,
            last_update_time_);
    local_raw_pub_.publish(target_);
    target_.header.stamp = ros::Time::now();
    last_update_time_ = target_.header.stamp;
}

void VelocityController::setupController(
    const CI& controller_idx,
    const std::string& name) 
{
    controllers_[static_cast<int>(controller_idx)].init(ros::NodeHandle(nh_, "velocity/" + name));
}

double VelocityController::computeEffort(
    const CI& controller_idx,
    const double& error,
    const ros::Time& last_time) 
{
    return 
        controllers_[static_cast<int>(controller_idx)].
            computeCommand(error, ros::Time::now() - last_time);
}

}