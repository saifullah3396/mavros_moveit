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

    target_.header.frame_id = "map";
    target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // set control mode
    std::string control_mode;
    p_nh.param<std::string>("control_mode", control_mode, "velocity");
    if (control_mode == "position") {
        control_mode_ = ControlMode::position;
        target_.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_VX | 
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_YAW |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_AFX | 
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ;
    } else if (control_mode == "velocity") {
        control_mode_ = ControlMode::velocity;
        velocity_control_handler_ = 
            std::unique_ptr<VelocityControlHandler>(
                new VelocityControlHandler(nh_)
            );
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

        typedef VelocityControlHandler::ControllerIndex CI;
        // Setup of the PID controllers
        for (int i = 0; i < static_cast<int>(CI::count)-1; ++i) {
            velocity_control_handler_->setupController(
                static_cast<CI>(i), 
                lin_rate_p, 
                lin_rate_i, 
                lin_rate_d, 
                lin_rate_i_max, 
                lin_rate_i_min);
        }
        velocity_control_handler_->setupController(
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

    // setup publishers/subscribers/services
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 5, &FollowMultiDofJointTrajectoryActionServer::stateCb, this);
    local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 5, &FollowMultiDofJointTrajectoryActionServer::poseCb, this);
    if (control_mode_ == ControlMode::position)
        local_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    else if (control_mode_ == ControlMode::velocity)
        local_vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // start the action server
    action_server_.start();

    ROS_INFO_STREAM("Initiated FollowMultiDofJointTrajectoryActionServer with control_mode: " << control_mode << " and rate: " << rate);
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
        last_update_time_ = ros::Time::now();

        //send a few setpoints before starting
        if (control_mode_ == ControlMode::position) {
            target_.position = current_pose_.pose.position;
            target_.yaw = getYaw(current_pose_.pose.orientation);
            local_pose_pub_.publish(target_);
            ros::spinOnce();
            rate_.sleep();
        } else if (control_mode_ == ControlMode::velocity) {
            target_.velocity.x = 0.0;
            target_.velocity.y = 0.0;
            target_.velocity.z = 0.0;
            target_.yaw_rate = 0.0;
            local_vel_pub_.publish(target_);
            ros::spinOnce();
            rate_.sleep();
        }

        // Arm vehicle
        if (!setArmRequest(true)) 
            action_server_.setAborted();

        // Set vehicle to offboard mode
        if (!setMavMode("OFFBOARD"))
            action_server_.setAborted();

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
            if (control_mode_ == ControlMode::position) {
                publishPositionCommand(cmd_pose.pose);
            } else if (control_mode_ == ControlMode::velocity) {
                publishVelocityCommand(cmd_pose.pose);
            }
            feedback_.current_pose = cmd_pose;
            action_server_.publishFeedback(feedback_);
            time += cycle_time_;
            last_update_time_ = ros::Time::now();
            rate_.sleep();
            ros::spinOnce();
        }

        /*tf::Pose final_pose;
        tf::poseMsgToTF(cmd_pose.pose, final_pose);
        while (!targetReached(final_pose)) {
            if(action_server_.isPreemptRequested() || !ros::ok()) {
                action_server_.setPreempted();
                success = false;
                break;
            }
            if (control_mode_ == ControlMode::position) {
                publishPositionCommand(cmd_pose.pose);
            } else if (control_mode_ == ControlMode::velocity) {
                publishVelocityCommand(cmd_pose.pose);
            }
            feedback_.current_pose = cmd_pose;
            action_server_.publishFeedback(feedback_);
            last_update_time_ = ros::Time::now();
            rate_.sleep();
            ros::spinOnce();
        }*/
    } else {
        ROS_WARN("Mavros not connected to FCU.");
        action_server_.setAborted();
        success = false;
    }

    // Set mode to loiter since keeping it in offboard requires sending commands
    // continuously
    setMavMode("AUTO.LOITER");

    if(success)
    {
        result_.error_code = Result::SUCCESSFUL;
        // set the action server to succeeded
        action_server_.setSucceeded(result_);
    }
}

bool FollowMultiDofJointTrajectoryActionServer::targetReached(const tf::Pose& target) {
    tf::Pose tf_curr;
    tf::poseMsgToTF(current_pose_.pose, tf_curr);
    auto diff_t = tf_curr.inverseTimes(target);
    auto yaw = getYaw(diff_t.getRotation());
    if (fabsf(diff_t.getOrigin().x()) <= target_pos_tol && 
        fabsf(diff_t.getOrigin().y()) <= target_pos_tol &&
        fabsf(diff_t.getOrigin().z()) <= target_pos_tol &&
        fabsf(yaw) <= target_orientation_tol)
    {
        return true;
    }
    return false;
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

void FollowMultiDofJointTrajectoryActionServer::publishPositionCommand(const geometry_msgs::Pose& cmd_pose) {
    target_.header.stamp = ros::Time::now();
    target_.position = cmd_pose.position;
    target_.yaw = getYaw(cmd_pose.orientation);
    local_pose_pub_.publish(target_);
}

void FollowMultiDofJointTrajectoryActionServer::publishVelocityCommand(const geometry_msgs::Pose& cmd_pose) {
    target_.header.stamp = ros::Time::now();
    typedef VelocityControlHandler::ControllerIndex CI;
    target_.velocity.x = 
        velocity_control_handler_->computeEffort(
            CI::x, 
            cmd_pose.position.x - current_pose_.pose.position.x, 
            last_update_time_);
    target_.velocity.y = 
        velocity_control_handler_->computeEffort(
            CI::y, 
            cmd_pose.position.y - current_pose_.pose.position.y, 
            last_update_time_);
    target_.velocity.z = 
        velocity_control_handler_->computeEffort(
            CI::z, 
            cmd_pose.position.z - current_pose_.pose.position.z, 
            last_update_time_);
    target_.yaw_rate =
        velocity_control_handler_->computeEffort(
            CI::yaw, 
            getYaw(cmd_pose.orientation) - getYaw(current_pose_.pose.orientation),
            last_update_time_);
    local_vel_pub_.publish(target_);
}

double FollowMultiDofJointTrajectoryActionServer::getYaw(const tf::Quaternion& q) const {
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double FollowMultiDofJointTrajectoryActionServer::getYaw(const geometry_msgs::Quaternion& q_msg) const {
    tf::Quaternion q;
    tf::quaternionMsgToTF(q_msg, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

bool FollowMultiDofJointTrajectoryActionServer::setMavMode(const std::string& mode) {
    if (current_state_.mode != mode) {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = mode;
        if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("Mode %s enabled.", mode.c_str());
            return true;
        } else {
            ROS_WARN("Mode %s could not be enabled. Cannot execute moveit trajectory.", mode.c_str());
            return false;
        }
    }
    return true;
}

bool FollowMultiDofJointTrajectoryActionServer::setArmRequest(const bool& arm) {
    if (current_state_.armed != arm) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            while (!current_state_.armed) { // Wait for arming to be complete
                ros::spinOnce();
                rate_.sleep();
            }
            return true;
        } else {
            ROS_WARN("Vehicle arm/disarm request failed. Cannot execute moveit trajectory.");
            return false;
        }
    }
    return true;
}

void FollowMultiDofJointTrajectoryActionServer::stateCb(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void FollowMultiDofJointTrajectoryActionServer::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FollowMultiDofJointTrajectoryActionServer");
  FollowMultiDofJointTrajectoryActionServer action_server("FollowMultiDofJointTrajectoryActionServer");
  action_server.init();
  action_server.idle();
  return 0;
}