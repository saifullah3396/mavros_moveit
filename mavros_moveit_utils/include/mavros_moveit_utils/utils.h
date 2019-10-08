#pragma once

#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <tf/tf.h>

namespace mavros_moveit_utils { 
  #define M_TWICE_PI 6.283185307

  template <typename T>
  T clamp(const T& n, const T& lower, const T& upper);

  template<typename T>
  T dist(const T& x1, const T& y1, const T& x2, const T& y2);

  template<typename T>
  T angleDist(const T& x1, const T& y1, const T& x2, const T& y2);

  template<typename T>
  T rangeToPi(const T& angle);

  template<typename T>
  T addAngles(const T& a1, const T& a2);
  
  template <typename T>
  T diffAngle(const T& a1, const T& a2);

  octomath::Vector3 tfToOctomath(const tf::Vector3& tf);
  double getYaw(const tf::Quaternion& q);
  double getYaw(const geometry_msgs::Quaternion& q_msg);
  tf::Vector3 getRPY(const tf::Quaternion& q);
}