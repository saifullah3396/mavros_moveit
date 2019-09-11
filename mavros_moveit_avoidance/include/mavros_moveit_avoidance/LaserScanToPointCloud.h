#pragma once

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class LaserScanToPointCloud 
{
public:
    LaserScanToPointCloud() {}
    ~LaserScanToPointCloud() {}

    void init() {
        std::string point_cloud_topic, laser_scan_topic;
        nh_.getParam("base_frame", base_frame_);
        nh_.getParam("laser_scan_topic", laser_scan_topic);
        nh_.getParam("point_cloud_topic", point_cloud_topic);
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 10);
        laser_scan_sub_ = nh_.subscribe(laser_scan_topic, 10, &LaserScanToPointCloud::scanCallback, this);
    }

private:
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
        if(!
            listener_.waitForTransform(
                laser_scan->header.frame_id,
                base_frame_,
                laser_scan->header.stamp + 
                ros::Duration().fromSec(laser_scan->ranges.size()*laser_scan->time_increment),
                ros::Duration(1.0)
            )) 
        {
            return;
        }
        sensor_msgs::PointCloud2 cloud;
        projector_.transformLaserScanToPointCloud(base_frame_, *laser_scan, cloud, listener_);
        point_cloud_pub_.publish(cloud);
    }

    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_sub_;
    ros::Publisher point_cloud_pub_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    std::string base_frame_;
};