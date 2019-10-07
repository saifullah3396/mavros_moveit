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
#include <mavros_moveit_actions/spline_interpolation.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>

class FollowMultiDofJointTrajectoryAction
{
    typedef actionlib::SimpleActionServer<mavros_moveit_actions::FollowMultiDofJointTrajectoryAction> ActionServer;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryResult Result;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryFeedback Feedback;
    typedef mavros_moveit_actions::FollowMultiDofJointTrajectoryGoalConstPtr GoalPtr;
public:
    FollowMultiDofJointTrajectoryAction(const std::string& name) : 
        action_name_(name), 
        action_server_(nh_, name, boost::bind(&FollowMultiDofJointTrajectoryAction::executeCb, this, _1), false)
    {
    }

    void init() {
        ros::NodeHandle p_nh("~");
        // setpoint publishing rate MUST be faster than 2Hz. From mavros documentation
        double rate;
		p_nh.param("rate", rate, 100.0);
        rate_ = ros::Rate(rate);
        cycle_time_ = rate_.expectedCycleTime().toSec();

        // set control mode
        std::string control_mode;
		p_nh.param<std::string>("control_mode", control_mode, "velocity");
        if (control_mode == "position")
            control_mode_ = ControlMode::position;
        else if (control_mode == "velocity") {
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
        }

        // setup publishers/subscribers/services
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &FollowMultiDofJointTrajectoryAction::stateCb, this);
        local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &FollowMultiDofJointTrajectoryAction::poseCb, this);
        if (control_mode_ == ControlMode::position)
            local_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
        else if (control_mode_ == ControlMode::velocity)
            local_vel_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // start the action server
        action_server_.start();

        ROS_INFO_STREAM("Initiated FollowMultiDofJointTrajectoryActionServer with control_mode: " << control_mode << " and rate: " << rate);
	}

    void idle() {
        while(ros::ok()){
            ros::spinOnce();
            ros::Rate(5).sleep();
        }
    }

    void executeCb(const GoalPtr &goal) {
        auto success = true;

        // check for FCU connection
        if (current_state_.connected) {
            // trajectory execution started
            executing = true;

            //send a few setpoints before starting
            if (control_mode_ == ControlMode::position) {
                mavros_msgs::PositionTarget target;
                target.position = current_pose_.pose.position;
                target.yaw = getYaw(current_pose_.pose.orientation);
                for(int i = 1; ros::ok() && i > 0; --i) {
                    local_pose_pub_.publish(target);
                    ros::spinOnce();
                    rate_.sleep();
                }
            } else if (control_mode_ == ControlMode::velocity) {
                mavros_msgs::PositionTarget target;
                target.velocity.x = 0.0;
                target.velocity.y = 0.0;
                target.velocity.z = 0.0;
                target.yaw_rate = 0.0;
                for(int i = 1; ros::ok() && i > 0; --i) {
                    local_vel_pub_.publish(target);
                    ros::spinOnce();
                    rate_.sleep();
                }
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
            last_update_time_ = ros::Time::now();
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
        executing = false;
    }

    bool targetReached(const tf::Pose& target) {
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

    CartesianInterpolation<double, 3> generateInterpolation(
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

    void publishPositionCommand(const geometry_msgs::Pose& cmd_pose) {
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.position = cmd_pose.position;
        target.yaw = getYaw(cmd_pose.orientation);
        local_pose_pub_.publish(target);
    }

    void publishVelocityCommand(const geometry_msgs::Pose& cmd_pose) {
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header.stamp = ros::Time::now();
        typedef VelocityControlHandler::ControllerIndex CI;
        target.velocity.x = 
            velocity_control_handler_->computeEffort(
                CI::x, 
                cmd_pose.position.x - current_pose_.pose.position.x, 
                last_update_time_);
        target.velocity.y = 
            velocity_control_handler_->computeEffort(
                CI::y, 
                cmd_pose.position.y - current_pose_.pose.position.y, 
                last_update_time_);
        target.velocity.z = 
            velocity_control_handler_->computeEffort(
                CI::z, 
                cmd_pose.position.z - current_pose_.pose.position.z, 
                last_update_time_);
        target.yaw_rate =
            velocity_control_handler_->computeEffort(
                CI::yaw, 
                getYaw(cmd_pose.orientation) - getYaw(current_pose_.pose.orientation),
                last_update_time_);
        local_vel_pub_.publish(target);
    }

    double getYaw(const tf::Quaternion& q) const {
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double getYaw(const geometry_msgs::Quaternion& q_msg) const {
        tf::Quaternion q;
        tf::quaternionMsgToTF(q_msg, q);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool setMavMode(const std::string& mode) {
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

    bool setArmRequest(const bool& arm) {
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

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

private:
    class VelocityControlHandler {
    public:
        VelocityControlHandler(const ros::NodeHandle& nh) : nh_(nh) 
        {
            controllers_.resize(static_cast<int>(ControllerIndex::count));
        }

        ~VelocityControlHandler() {}

        enum class ControllerIndex {
            x, y, z, yaw, count
        };

        void setupController(
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

        double computeEffort(
            const ControllerIndex& controller_idx,
            const double& error,
            const ros::Time& last_time) 
        {
            return 
                controllers_[static_cast<int>(controller_idx)].
                    computeCommand(error, ros::Time::now() - last_time);
        }

    private:
        std::vector<control_toolbox::Pid> controllers_;        
        ros::NodeHandle nh_; // node handle
    };

    ros::NodeHandle nh_; // node handle

    ActionServer action_server_; // simple actionlib server
    Feedback feedback_; // action server feedback
    Result result_; // action server result
    std::string action_name_; // action name
    ros::Time last_update_time_; // last update time
    double cycle_time_;

    mavros_msgs::State current_state_; // latest mavros state
    geometry_msgs::PoseStamped current_pose_; // latest robot pose
    ros::Rate rate_ = {ros::Rate(20.0)};  // ros run rate
    ros::Subscriber local_pose_sub_; // mavros local position subscriber
    ros::Publisher local_pose_pub_; // mavros position commands publisher
    ros::Publisher local_vel_pub_; // mavros velocity commands publisher
    ros::Subscriber state_sub_; // mavros state subscriber 
    ros::ServiceClient arming_client_; // mavros service for arming/disarming the robot
    ros::ServiceClient set_mode_client_; // mavros service for setting mode. Position commands are only available in mode OFFBOARD.

    bool executing = {false}; // whether the action server is currently in execution
    
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FollowMultiDofJointTrajectoryActionServer");
  FollowMultiDofJointTrajectoryAction action_server("FollowMultiDofJointTrajectoryAction");
  action_server.init();
  action_server.idle();
  return 0;
}