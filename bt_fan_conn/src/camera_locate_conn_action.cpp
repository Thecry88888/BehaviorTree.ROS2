#include "bt_fan_conn/camera_locate_conn_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool CameraLocateConnAction::setGoal(Goal& goal)
{
    goal.start_detection = true;
    return true;
}

NodeStatus CameraLocateConnAction::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived.Done = %s", name().c_str(),
              wr.result->success ? "true" : "false");

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CameraLocateConnAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void CameraLocateConnAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class CameraLocateConnAction will self register with name  "CameraLocateConnAction".
CreateRosNodePlugin(CameraLocateConnAction, "CameraLocateConnAction");