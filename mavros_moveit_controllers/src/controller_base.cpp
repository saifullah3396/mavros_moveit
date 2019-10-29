#include <tf/tf.h>
#include <mavros_moveit_controllers/controller_base.h>
#include <mavros_moveit_controllers/position_controller.h>
#include <mavros_moveit_controllers/velocity_controller.h>
#include <mavros_moveit_utils/utils.h>

namespace mavros_moveit_controllers {

void ControllerBase::init() 
{
    ros::NodeHandle p_nh("~");
    // setpoint publishing rate MUST be faster than 2Hz. From mavros documentation
    double rate;
    p_nh.param("rate", rate, 100.0);
    rate_ = ros::Rate(rate);
    std::string frame_id;
    p_nh.param<std::string>("frame_id", frame_id, "map");
    target_.header.frame_id = frame_id;
    int coordinate_frame;
    p_nh.param("coordinate_frame", coordinate_frame, (int)mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
    target_.coordinate_frame = coordinate_frame;

    // setup publishers/subscribers
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 5, &ControllerBase::stateCb, this);
    local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 5, &ControllerBase::poseCb, this);
    local_cmd_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros_moveit/local_position/cmd_pose", 5, &ControllerBase::commandCb, this);
    local_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);

    // services
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_offboard_server_ = nh_.advertiseService("mavros_moveit/set_offboard", &ControllerBase::setOffboard, this);
}

ControllerBase* ControllerBase::makeController(const std::string& type) {
    ControllerBase* controller;
    if (type == "position")
        controller = new PositionController();
    else if (type == "velocity")
        controller = new VelocityController();
    else
        ROS_FATAL(
            "Cannot initialize controller of type '%s'. \
            Possible types are: 'position' and 'velocity'", type.c_str());
    return controller;
}

void ControllerBase::update() {
    while(ros::ok()){
        if (statusCheck()) {
            if (last_cmd_pose_.header.stamp.toSec() > 0) {
                generateCommand(last_cmd_pose_);
            }
        }
        ros::spinOnce();
        rate_.sleep();
    }
}

bool ControllerBase::statusCheck()
{
    // is mavros connected to px4?
    if (!conditionCheck(
        mavros_state_.connected, 
        "Controller cannot generate command since mavros is not connected to px4",
        connected_status_wait_flag_))
    {
        return false;
    }
        
    // robot state is received yet?
    if (!conditionCheck(
        state_received_, 
        "Controller cannot generate command due to unknown mavros state",
        state_received_wait_flag_))
    {
        return false;
    }

     // robot current pose is received yet?
    if (!conditionCheck(
        pose_received_, 
        "Controller cannot generate command due to unknown robot pose",
        pose_received_wait_flag_))
    {
        return false;
    }

    // is robot armed?
    if (!conditionCheck(
        mavros_state_.armed, 
        "Controller cannot generate command due to unarmed robot",
        unarmed_wait_flag_))
    {
        return false;
    }

    if (!conditionCheck(
        mavros_state_.mode == "OFFBOARD", 
        "Controller cannot generate command unless robot mode is set to OFFBOARD",
        wrong_mode_wait_flag_))
    {
        return false;
    }
    return true;
}


bool ControllerBase::conditionCheck(
    const bool& condition, 
    const std::string& msg,
    bool& wait_flag)
{
    if (!condition) { // condition false
        if (!wait_flag) {
            ROS_WARN(msg.c_str());
            wait_flag = true;
        }
        return false;
    } else {
        wait_flag = false;
        return true;
    }
}

bool ControllerBase::targetReached(const tf::Pose& target) {
    tf::Pose tf_curr;
    tf::poseMsgToTF(current_pose_.pose, tf_curr);
    auto diff_t = tf_curr.inverseTimes(target);
    auto yaw = mavros_moveit_utils::getYaw(diff_t.getRotation());
    if (fabsf(diff_t.getOrigin().x()) <= target_pos_tol_ && 
        fabsf(diff_t.getOrigin().y()) <= target_pos_tol_ &&
        fabsf(diff_t.getOrigin().z()) <= target_pos_tol_ &&
        fabsf(yaw) <= target_orientation_tol_)
    {
        return true;
    }
    return false;
}

bool ControllerBase::setOffboard(
  mavros_moveit_controllers::SetOffboard::Request& req, // dummy res,req
  mavros_moveit_controllers::SetOffboard::Response& res)
{
    // send one command
    generateCommand(current_pose_);
    ros::spinOnce();
    rate_.sleep();

    // Set vehicle to offboard mode
    return mavros_moveit_utils::setMavMode(mavros_state_, "OFFBOARD", set_mode_client_);
}

void ControllerBase::stateCb(const mavros_msgs::State::ConstPtr& mavros_state) {
    mavros_state_ = *mavros_state;
    state_received_ = true;
}

void ControllerBase::poseCb(const geometry_msgs::PoseStamped::ConstPtr& current_pose) {
    current_pose_ = *current_pose;
    pose_received_ = true;
}

void ControllerBase::commandCb(const geometry_msgs::PoseStamped::ConstPtr& cmd_pose) {
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(cmd_pose->pose.orientation, orientation);
    if (fabsf(orientation.length() - 1.0) < 1e-5) {// invalid orientation command
        last_cmd_pose_ = *cmd_pose;
        last_cmd_pose_.header.stamp = ros::Time::now();
    } else {
        ROS_WARN("Invalid orientation command given to controller");
    }
}

}