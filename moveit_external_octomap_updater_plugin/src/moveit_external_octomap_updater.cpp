/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Cyrille Morin
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Jon Binney, Ioan Sucan, Cyrille Morin */

#include <pluginlib/class_list_macros.h>
#include <moveit_external_octomap_updater/moveit_external_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/conversions.h>
#include <XmlRpcException.h>

namespace occupancy_map_monitor
{
ExternalOctomapUpdater::ExternalOctomapUpdater()
  : OccupancyMapUpdater("ExternalUpdater")
  , private_nh_("~")
  , octomap_subscriber_(NULL)
{
}

ExternalOctomapUpdater::~ExternalOctomapUpdater()
{
  stopHelper();
}

bool ExternalOctomapUpdater::setParams(XmlRpc::XmlRpcValue& params)
{
  try
  {
    if (!params.hasMember("octomap_topic"))
      return false;
    octomap_topic = static_cast<const std::string&>(params["octomap_topic"]);

  }
  catch (XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool ExternalOctomapUpdater::initialize()
{
  return true;
}

void ExternalOctomapUpdater::start()
{
  if (octomap_subscriber_)
    return;
  /* subscribe to octomap topic using tf filter*/
  octomap_subscriber_ = new message_filters::Subscriber<octomap_msgs::Octomap>(root_nh_, octomap_topic, 5);

  octomap_subscriber_->registerCallback(boost::bind(&ExternalOctomapUpdater::octoMsgCallback, this, _1));
  ROS_INFO("Listening to '%s'", octomap_topic.c_str());

}

void ExternalOctomapUpdater::stopHelper()
{
  delete octomap_subscriber_;
}

void ExternalOctomapUpdater::stop()
{
  stopHelper();
  octomap_subscriber_ = NULL;
}

ShapeHandle ExternalOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  ROS_WARN("Shape filter not used by the ExternalOctomapUpdater!");
  return 0;
}

void ExternalOctomapUpdater::forgetShape(ShapeHandle handle)
{
  ROS_WARN("Shape filter not used by the ExternalOctomapUpdater!");
}



void ExternalOctomapUpdater::octoMsgCallback(const octomap_msgs::Octomap::ConstPtr& octo_msg)
{
  tree_->lockWrite();

  tree_->clear();
  octomap_msgs::readTree<octomap::OcTree>( tree_.get(),*octo_msg); //Fill the octomap with the content of the message

  tree_->unlockWrite();

  tree_->triggerUpdateCallback();

}
}
PLUGINLIB_EXPORT_CLASS(occupancy_map_monitor::ExternalOctomapUpdater, occupancy_map_monitor::OccupancyMapUpdater)
