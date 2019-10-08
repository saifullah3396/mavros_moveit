#include <vector>
#include <ros/ros.h>
#include <control_toolbox/pid.h>

namespace mavros_moveit_controllers {

class ControllerBase;
class PositionController : public ControllerBase {
public:
    PositionController();
    virtual ~PositionController();

    void init() override; // Initiates the action server
    void commandCb(const geometry_msgs::PoseStamped::ConstPtr& cmd_pose) override; // command pose callback
};

}