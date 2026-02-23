#include "behaviortree_ros2/bt_action_node.hpp"
#include "moveit_msgs/action/move_group.hpp"

using namespace BT;

class FollowPathAction : public RosActionNode<moveit_msgs::action::MoveGroup>
{
public:
    FollowPathAction(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
        : RosActionNode<moveit_msgs::action::MoveGroup>(name, conf, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<trajectory_msgs::msg::JointTrajectory>("path")
        });
    }

    bool setGoal(Goal& goal) override;

    void onHalt() override;

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};