#include <mavros_moveit_controllers/controller_base.h>
#include <mavros_moveit_controllers/position_controller.h>
#include <mavros_moveit_controllers/velocity_controller.h>

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

    // setup publishers/subscribers/services
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 5, &ControllerBase::stateCb, this);
    local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 5, &ControllerBase::poseCb, this);
    local_cmd_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/cmd_pose", 5, &ControllerBase::poseCb, this);
    local_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
}

void ControllerBase::update() {
    while(ros::ok()){
        ros::spinOnce();
        rate_.sleep();
    }
}

bool ControllerBase::statusCheck()
{
    // robot state is received yet?
    if (!state_received_) {
        ROS_WARN("Controller cannot generate command due to unknown mavros state.");
        return false;
    }

    // robot current pose is received yet?
    if (!pose_received_) {
        ROS_WARN("Controller cannot generate command due to unknown robot pose.");
        return false;
    }

    // is robot armed?
    if (!mavros_state_.armed) {
        ROS_WARN("Controller cannot generate command due to unarmed robot.");
        return false;
    }

    // is robot armed?
    if (!mavros_state_.armed) {
        ROS_WARN("Controller cannot generate command due to unarmed robot.");
        return false;
    }

    if (mavros_state_.mode != "OFFBOARD") {
        ROS_WARN("Controller cannot generate command unless robot mode is set to OFFBOARD.");
        return false;
    }
    return true;
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

void ControllerBase::stateCb(const mavros_msgs::State::ConstPtr& mavros_state) {
    mavros_state_ = *mavros_state;
    state_received_ = true;
}

void ControllerBase::poseCb(const geometry_msgs::PoseStamped::ConstPtr& current_pose) {
    current_pose_ = *current_pose;
    pose_received_ = true;
}

}