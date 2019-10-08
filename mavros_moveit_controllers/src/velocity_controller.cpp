#include <mavros_moveit_controllers/controller_base.h>
#include <mavros_moveit_controllers/velocity_controller.h>
#include <mavros_moveit_utils/utils.h>

namespace mavros_moveit_controllers {

VelocityController::VelocityController()
{
    controllers_.resize(static_cast<int>(ControllerIndex::count));
}

VelocityController::~VelocityController() {}

void VelocityController::init()
{
    // Initialize base parameters
    ControllerBase::init();

    ros::NodeHandle p_nh("~");
    double lin_rate_p, lin_rate_i, lin_rate_d, lin_rate_i_min, lin_rate_i_max;
    double yaw_rate_p, yaw_rate_i, yaw_rate_d, yaw_rate_i_min, yaw_rate_i_max;

    // Linear velocity PID gains and bound of integral windup
    p_nh.param("lin_rate_p", lin_rate_p, 0.4);
    p_nh.param("lin_rate_i", lin_rate_i, 0.05);
    p_nh.param("lin_rate_d", lin_rate_d, 0.12);
    p_nh.param("lin_rate_i_min", lin_rate_i_min, -0.1);
    p_nh.param("lin_rate_i_max", lin_rate_i_max, 0.1);

    // Yaw rate PID gains and bounds of integral windup
    p_nh.param("yaw_rate_p", yaw_rate_p, 0.011);
    p_nh.param("yaw_rate_i", yaw_rate_i, 0.00058);
    p_nh.param("yaw_rate_d", yaw_rate_d, 0.12);
    p_nh.param("yaw_rate_i_min", yaw_rate_i_min, -0.005);
    p_nh.param("yaw_rate_i_max", yaw_rate_i_max, 0.005);

    // Setup of the PID controllers
    for (int i = 0; i < static_cast<int>(CI::count)-1; ++i) {
        setupController(
            static_cast<CI>(i), 
            lin_rate_p, 
            lin_rate_i, 
            lin_rate_d, 
            lin_rate_i_max, 
            lin_rate_i_min);
    }
    setupController(
        CI::yaw, 
        yaw_rate_p, 
        yaw_rate_i, 
        yaw_rate_d, 
        yaw_rate_i_max, 
        yaw_rate_i_min);

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
    target_.header.stamp = ros::Time::now();
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
    target_.yaw_rate =
        computeEffort(
            CI::yaw, 
            mavros_moveit_utils::getYaw(cmd_pose.pose.orientation) - 
            mavros_moveit_utils::getYaw(current_pose_.pose.orientation),
            last_update_time_);
    local_raw_pub_.publish(target_);
    last_update_time_ = ros::Time::now();
}

void VelocityController::setupController(
    const ControllerIndex& controller_idx,
    const double& p,
    const double& i,
    const double& d,
    const double& i_min,
    const double& i_max) 
{
    #ifdef CONTROL_TOOLBOX_PRE_1_14
        controllers_[static_cast<int>(controller_idx)].initPid(p, i, d, i_min, i_max, nh_);
    #else
        controllers_[static_cast<int>(controller_idx)].initPid(p, i, d, i_min, i_max, false, nh_);
    #endif
}

double VelocityController::computeEffort(
    const ControllerIndex& controller_idx,
    const double& error,
    const ros::Time& last_time) 
{
    return 
        controllers_[static_cast<int>(controller_idx)].
            computeCommand(error, ros::Time::now() - last_time);
}

}