#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/locate_object.hpp"

using namespace BT;

class CameraLocateFanAction : public RosActionNode<btcpp_ros2_interfaces::action::LocateObject>
{
public:
  CameraLocateFanAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::LocateObject>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
        OutputPort<geometry_msgs::msg::Pose>("camera_fan_detected_pose")});
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
