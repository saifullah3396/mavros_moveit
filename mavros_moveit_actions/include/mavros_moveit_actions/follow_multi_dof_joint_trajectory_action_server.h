/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Saifullah */

#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_toolbox/pid.h>
#include <mavros_moveit_actions/FollowMultiDofJointTrajectoryAction.h>
#include <mavros_moveit_actions/cartesian_interpolation.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>

#include <mavros_moveit_actions/velocity_control_handler.h>

class FollowMultiDofJointTrajectoryActionServer
{
    typedef actionlib::SimpleActionServer<mavros_moveit_actions::FollowMultiDofJointTrajectoryAction> ActionServer;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryResult Result;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryFeedback Feedback;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryGoalConstPtr GoalPtr;
public:
    FollowMultiDofJointTrajectoryActionServer(const std::string& name); // constructor
    void init(); // Initiates the action server
    void idle(); // Action server update cycle in idle state
    void executeCb(const GoalPtr &goal); // Execute callback of action server

    // Generates an interpolated trajectory from given position and orientation waypoints
    CartesianInterpolation<double, 3> generateInterpolation(
        const trajectory_msgs::MultiDOFJointTrajectory& trajectory,
        Eigen::Matrix<double, Eigen::Dynamic, 1>& knots);
      
    // publishes position command for given target pose
    void publishPositionCommand(const geometry_msgs::Pose& cmd_pose); 

    // publishes velocity command for given target pose
    void publishVelocityCommand(const geometry_msgs::Pose& cmd_pose);
    bool targetReached(const tf::Pose& target); // Checkes if the target is within tolerance
    double getYaw(const tf::Quaternion& q) const; // Returns yaw of quaternion
    double getYaw(const geometry_msgs::Quaternion& q_msg) const; // Returns yaw of quaternion msg
    bool setMavMode(const std::string& mode); // Sets mavros mode to given mode
    bool setArmRequest(const bool& arm); // Arms or disarms the robot

    // ros callbacks
    void stateCb(const mavros_msgs::State::ConstPtr& msg); // mavros state callback
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg); // mavros pose callback

private:
    // ros
    ros::NodeHandle nh_; // node handle
    ros::Rate rate_ = {ros::Rate(20.0)};  // ros run rate

    // subscribers
    ros::Subscriber local_pose_sub_; // mavros local position subscriber
    ros::Subscriber state_sub_; // mavros state subscriber 

    // publishers
    ros::Publisher local_pose_pub_; // mavros position commands publisher
    ros::Publisher local_vel_pub_; // mavros velocity commands publisher

    // services
    ros::ServiceClient arming_client_; // mavros service for arming/disarming the robot
    ros::ServiceClient set_mode_client_; // mavros service for setting mode. Position commands are only available in mode OFFBOARD.

    // action server
    ActionServer action_server_; // simple actionlib server
    Feedback feedback_; // action server feedback
    Result result_; // action server result
    std::string action_name_; // action name

    // control variables
    mavros_msgs::PositionTarget target_; // mavros target container
    mavros_msgs::State current_state_; // latest mavros state
    geometry_msgs::PoseStamped current_pose_; // latest robot pose
    ros::Time last_update_time_; // last update time
    double cycle_time_;

    const float curr_to_start_pose_time_ = 0.25; // Time for moving from initial pose to start planning pose
    const float target_pos_tol = {1e-1}; // difference tolerance of position from the target position
    const float target_orientation_tol = {5e-2}; // // difference tolerance of orientation from the target orientation

    typedef std::unique_ptr<VelocityControlHandler> VelocityControlHandlerPtr;
    VelocityControlHandlerPtr velocity_control_handler_; // Velocity control handler

    enum class ControlMode {
        position,
        velocity
    };
    ControlMode control_mode_ = {ControlMode::position};
};