#include <vector>
#include <ros/ros.h>
#include <control_toolbox/pid.h>

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