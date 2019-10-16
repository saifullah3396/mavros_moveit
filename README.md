# MAVROS MOVEIT

Moveit controller manager and configuration for integration with Mavros. 

## Getting Started

### Prerequisites

- The build is only yet tested on ROS melodic but it should work on other distributions as well.

- The package depends on px4 integration with gazebo simulator and it must be installed from source in a local catkin workspace. Detailed instructions for installing px4 on linux can be found at https://dev.px4.io/master/en/simulation/gazebo.html.

- Another major dependecy of the package is mavros. Installation instructions for mavros can be found at https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html.

- Install moveit dependencies for given ROS distribution:

```
sudo apt-get install ros-<ros-distro>-moveit-*
```

### Installation

- Make a local cakin workspace:
```
mkdir -p ~/catkin_ws/src
```
- Clone the repository to your workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/saifullah3396/mavros-moveit.git
```
- Build and source the workspace:
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Test mavros_moveit

```
- Run px4, mavros moveit using the launch file. The example uses iris but can be updated for use with other vehicles:
```
roslaunch mavros_moveit px4_mavros_moveit.launch
```

### Test mavros_moveit_avoidance with depth camera sensor
```
roslaunch mavros_moveit_avoidance px4_indoor_iris_depth.launch
```

### Test mavros_moveit_avoidance with rplidar sensor
```
roslaunch mavros_moveit_avoidance px4_indoor_iris_rplidar.launch
```
