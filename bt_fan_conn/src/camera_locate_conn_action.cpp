#include "bt_fan_conn/camera_locate_conn_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool CameraLocateConnAction::setRequest(std::shared_ptr<Request>& request)
{
    request->start_detection = true;
    return true;
}

NodeStatus CameraLocateConnAction::onResponseReceived(const Response::SharedPtr& response)
{
    if (response->success) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

// Plugin registration.
// The class CameraLocateConnAction will self register with name  "CameraLocateConnAction".
CreateRosNodePlugin(CameraLocateConnAction, "CameraLocateConnAction");