#include "bt_fan_conn/camera_locate_fan_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool CameraLocateFanAction::setGoal(Goal& goal)
{
    goal.start_detection = true;
    return true;
}

NodeStatus CameraLocateFanAction::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived.Done = %s", name().c_str(),
              wr.result->success ? "true" : "false");

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CameraLocateFanAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void CameraLocateFanAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class CameraLocateFanAction will self register with name  "CameraLocateFanAction".
CreateRosNodePlugin(CameraLocateFanAction, "CameraLocateFanAction");