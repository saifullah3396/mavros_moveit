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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>

#include <mavros_moveit_utils/utils.h>
#include <mavros_moveit_actions/follow_multi_dof_joint_trajectory_action_server.h>

FollowMultiDofJointTrajectoryActionServer::FollowMultiDofJointTrajectoryActionServer(const std::string& name) : 
    action_name_(name), 
    action_server_(nh_, name, boost::bind(&FollowMultiDofJointTrajectoryActionServer::executeCb, this, _1), false)
{
}

void FollowMultiDofJointTrajectoryActionServer::init() {
    ros::NodeHandle p_nh("~");
    // setpoint publishing rate MUST be faster than 2Hz. From mavros documentation
    double rate;
    p_nh.param("rate", rate, 100.0);
    rate_ = ros::Rate(rate);
    cycle_time_ = rate_.expectedCycleTime().toSec();

    std::string control_mode;
    p_nh.param<std::string>("control_mode", control_mode, "velocity");

    state_sub_ = 
        nh_.subscribe<mavros_msgs::State>(
            "mavros/state", 10, &FollowMultiDofJointTrajectoryActionServer::stateCb, this);
    local_pose_sub_ = 
        nh_.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 10, &FollowMultiDofJointTrajectoryActionServer::poseCb, this);
    local_cmd_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros_moveit/local_position/cmd_pose", 5);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_offboard_client_ = nh_.serviceClient<mavros_moveit_controllers::SetOffboard>("mavros_moveit/set_offboard");

    // start the action server
    action_server_.start();
}

void FollowMultiDofJointTrajectoryActionServer::idle() {
    while(ros::ok()){
        ros::spinOnce();
        ros::Rate(5).sleep();
    }
}

void FollowMultiDofJointTrajectoryActionServer::executeCb(const GoalPtr &goal) {
    auto success = true;

    // check for FCU connection
    if (current_state_.connected) {
        // Arm vehicle
        if (!mavros_moveit_utils::setArmRequest(current_state_, true, arming_client_))
            action_server_.setAborted();

        mavros_moveit_controllers::SetOffboard set_offboard_msg;
        if (!set_offboard_client_.call(set_offboard_msg))
            action_server_.setAborted();
        ros::spinOnce();

        auto& trajectory = goal->trajectory;
        Eigen::Matrix<double, Eigen::Dynamic, 1> knots;
        auto spline_interpolation = generateInterpolation(trajectory, knots);
        auto time = 0.0;
        geometry_msgs::PoseStamped cmd_pose;
        while (time <= knots.tail(1)[0]) {
            if(action_server_.isPreemptRequested() || !ros::ok()) {
                action_server_.setPreempted();
                success = false;
                break;
            }
            Eigen::Matrix<double, 3, 1> interp_position = (*spline_interpolation.position)(time);
            Eigen::Quaternion<double> interp_orientation = (*spline_interpolation.orientation)(time);
            cmd_pose.pose.position.x = interp_position[0];
            cmd_pose.pose.position.y = interp_position[1];
            cmd_pose.pose.position.z = interp_position[2];
            cmd_pose.pose.orientation.w = interp_orientation.w();
            cmd_pose.pose.orientation.x = interp_orientation.x();
            cmd_pose.pose.orientation.y = interp_orientation.y();
            cmd_pose.pose.orientation.z = interp_orientation.z();
            local_cmd_pose_pub_.publish(cmd_pose);
            feedback_.current_pose = cmd_pose;
            action_server_.publishFeedback(feedback_);
            time += cycle_time_;
            rate_.sleep();
            ros::spinOnce();
        }
    } else {
        ROS_WARN("Mavros not connected to FCU.");
        action_server_.setAborted();
        success = false;
    }

    // Set mode to loiter since keeping it in offboard requires sending commands
    // continuously
    if (!mavros_moveit_utils::setMavMode(current_state_, "AUTO.LOITER", set_mode_client_))
        action_server_.setAborted();

    if(success)
    {
        result_.error_code = Result::SUCCESSFUL;
        // set the action server to succeeded
        action_server_.setSucceeded(result_);
    }
}

CartesianInterpolation<double, 3> FollowMultiDofJointTrajectoryActionServer::generateInterpolation(
    const trajectory_msgs::MultiDOFJointTrajectory& trajectory,
    Eigen::Matrix<double, Eigen::Dynamic, 1>& knots)
{
    auto size = trajectory.points.size() + 1;
    Eigen::Matrix<double, Eigen::Dynamic, 3> positions(size, 3);
    std::vector<Eigen::Quaternion<double> > orientations(size);
    knots.resize(size);
    knots[0] = 0.0;
    auto& trans = current_pose_.pose.position;
    auto& rot = current_pose_.pose.orientation;
    positions.row(0) << trans.x, trans.y, trans.z;
    orientations[0] = Eigen::Quaternion<double>(rot.w, rot.x, rot.y, rot.z);
    auto cycle_time = rate_.expectedCycleTime().toSec();
    for (int i = 1; i < size; ++i) {
        auto& trans = trajectory.points[i-1].transforms[0].translation;
        auto& rot = trajectory.points[i-1].transforms[0].rotation;
        positions.row(i) << trans.x, trans.y, trans.z;
        orientations[i] = Eigen::Quaternion<double>(rot.w, rot.x, rot.y, rot.z);
        knots[i] = trajectory.points[i-1].time_from_start.toSec() + curr_to_start_pose_time_;
    }
    #ifdef DEBUG
    ROS_DEBUG_STREAM("Interpolation positions:" << positions);
    for (int i = 0; i < size; ++i) {
        ROS_DEBUG_STREAM("Interpolation orientations:" << 
            orientations[i].w() << ", " << 
            orientations[i].x() << ", " << 
            orientations[i].y() << ", " << 
            orientations[i].z());
    }
    ROS_DEBUG_STREAM("Interpolation knots:" << knots.transpose());
    #endif
    return CartesianInterpolation<double, 3>(positions, orientations, knots);
}

void FollowMultiDofJointTrajectoryActionServer::stateCb(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void FollowMultiDofJointTrajectoryActionServer::poseCb(const geometry_msgs::PoseStamped::ConstPtr& current_pose) {
    current_pose_ = *current_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FollowMultiDofJointTrajectoryActionServer");
  FollowMultiDofJointTrajectoryActionServer action_server("FollowMultiDofJointTrajectoryActionServer");
  action_server.init();
  action_server.idle();
  return 0;
}