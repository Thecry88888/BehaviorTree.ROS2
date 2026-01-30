#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/move_to.hpp"

using namespace BT;

class MoveToAction : public RosActionNode<btcpp_ros2_interfaces::action::MoveTo>
{
public:
  MoveToAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::MoveTo>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ 
        InputPort<geometry_msgs::msg::Pose>("target_pose"),
        InputPort<int8_t>("velocity_override") });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
