#include "behaviortree_ros2/bt_service_node.hpp"
#include "btcpp_ros2_interfaces/srv/locate_object.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace BT;

class TactileSensorAction : public RosServiceNode<btcpp_ros2_interfaces::srv::LocateObject>
{
public:
  TactileSensorAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosServiceNode<btcpp_ros2_interfaces::srv::LocateObject>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
        OutputPort<geometry_msgs::msg::Pose>("sensor_connector_detected_pose")});
  }

  bool setRequest(std::shared_ptr<Request>& request) override;

  NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
};
