#include <vector>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

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
    virtual void commandCb(const geometry_msgs::PoseStamped::ConstPtr& cmd_pose) = 0; // command pose callback

protected:
    ros::NodeHandle nh_; // node handle
    ros::Rate rate_ = {ros::Rate(100.0)};  // ros run rate

    // subscribers
    ros::Subscriber local_pose_sub_; // mavros local position subscriber
    ros::Subscriber local_cmd_pose_sub_; // mavros local commanded position subscriber
    ros::Subscriber state_sub_; // mavros state subscriber 

    // publishers
    ros::Publisher local_raw_pub_; // mavros raw commands publisher

    // subscriber checks
    bool state_received_ = {false};
    bool pose_received_ = {false};

    // control variables
    mavros_msgs::PositionTarget target_; // mavros target container
    mavros_msgs::State mavros_state_; // latest mavros state
    geometry_msgs::PoseStamped current_pose_; // latest robot pose
    ros::Time last_update_time_; // last update time
};

}