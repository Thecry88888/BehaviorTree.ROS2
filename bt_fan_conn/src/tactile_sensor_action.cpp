#include "bt_fan_conn/tactile_sensor_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool TactileSensorAction::setRequest(std::shared_ptr<Request>& request)
{
    request->start_detection = true;
    return true;
}

NodeStatus TactileSensorAction::onResponseReceived(const Response::SharedPtr& response)
{
    if (response->success) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

// Plugin registration.
// The class TactileSensorAction will self register with name  "TactileSensorAction".
CreateRosNodePlugin(TactileSensorAction, "TactileSensorAction");