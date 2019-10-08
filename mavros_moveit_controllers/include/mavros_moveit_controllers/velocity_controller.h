#include <vector>
#include <ros/ros.h>
#include <control_toolbox/pid.h>

namespace mavros_moveit_controllers {

class ControllerBase;
class VelocityController : public ControllerBase {
    enum class ControllerIndex;
    typedef ControllerIndex CI;
public:
    VelocityController();
    virtual ~VelocityController();

    void init() override; // Initiates the action server
    void generateCommand(const geometry_msgs::PoseStamped& cmd_pose) override; // command pose callback

    void setupController(
        const CI& controller_idx,
        const double& p,
        const double& i,
        const double& d,
        const double& i_min,
        const double& i_max);

    double computeEffort(
        const CI& controller_idx,
        const double& error,
        const ros::Time& last_time);

private:
    std::vector<control_toolbox::Pid> controllers_;

    enum class ControllerIndex {
        x, y, z, yaw, count
    };    
};

}