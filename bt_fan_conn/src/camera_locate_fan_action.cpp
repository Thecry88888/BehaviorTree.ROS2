#include "bt_fan_conn/camera_locate_fan_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool CameraLocateFanAction::setRequest(std::shared_ptr<Request>& request)
{
    request->start_detection = true;
    return true;
}

NodeStatus CameraLocateFanAction::onResponseReceived(const Response::SharedPtr& response)
{
    if (response->success) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

// Plugin registration.
// The class CameraLocateFanAction will self register with name  "CameraLocateFanAction".
CreateRosNodePlugin(CameraLocateFanAction, "CameraLocateFanAction");