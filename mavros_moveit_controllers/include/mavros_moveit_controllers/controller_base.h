#include <vector>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_moveit_controllers/SetOffboard.h>

namespace mavros_moveit_controllers {

class ControllerBase {
public:
    ControllerBase() {}
    virtual ~ControllerBase() {}
    virtual void init(); // Initiates the action server
    void update(); // controller update
    static ControllerBase* makeController(const std::string& type);
    
    // ros callbacks
    void stateCb(const mavros_msgs::State::ConstPtr& msg); // mavros state callback
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg); // mavros pose callback
    void commandCb(const geometry_msgs::PoseStamped::ConstPtr& cmd_pose); // command pose callback
    bool setOffboard(
        mavros_moveit_controllers::SetOffboard::Request& req,
        mavros_moveit_controllers::SetOffboard::Response& res);

protected:
    bool statusCheck();
    virtual void generateCommand(const geometry_msgs::PoseStamped& cmd_pose) = 0; // command generation
    bool targetReached(const tf::Pose& target);

    ros::NodeHandle nh_; // node handle
    ros::Rate rate_ = {ros::Rate(100.0)};  // ros run rate

    // subscribers
    ros::Subscriber local_pose_sub_; // mavros local position subscriber
    ros::Subscriber local_cmd_pose_sub_; // mavros local commanded position subscriber
    ros::Subscriber state_sub_; // mavros state subscriber 

    // publishers
    ros::Publisher local_raw_pub_; // mavros raw commands publisher

    // service
    ros::ServiceServer set_offboard_server_;
    ros::ServiceClient set_mode_client_; // mavros service for setting mode.

    // subscriber checks
    bool state_received_ = {false};
    bool pose_received_ = {false};

    // control variables
    mavros_msgs::PositionTarget target_; // mavros target container
    mavros_msgs::State mavros_state_; // latest mavros state
    geometry_msgs::PoseStamped current_pose_; // latest robot pose
    ros::Time last_update_time_; // last update time

    const double target_pos_tol = {1e-1}; // difference tolerance of position from the target position
    const double target_orientation_tol = {5e-2}; // // difference tolerance of orientation from the target orientation
};

}