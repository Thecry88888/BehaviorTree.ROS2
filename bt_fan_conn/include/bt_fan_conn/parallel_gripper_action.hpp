#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/parallel_gripper.hpp"

using namespace BT;

class ParallelGripperAction : public RosActionNode<btcpp_ros2_interfaces::action::ParallelGripper>
{
public:
  ParallelGripperAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::ParallelGripper>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ InputPort<int8_t>("command") });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
