#include <mavros_moveit_avoidance/LaserScanToPointCloud.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "laser_to_pc_node");
    LaserScanToPointCloud laser_scan_to_point_cloud;
    laser_scan_to_point_cloud.init();
    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}