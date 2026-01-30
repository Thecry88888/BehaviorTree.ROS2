#include "bt_fan_conn/move_to_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool MoveToAction::setGoal(RosActionNode::Goal& goal)
{
    auto target_pose = getInput<geometry_msgs::msg::Pose>("target_pose");
    if (!target_pose) {
        RCLCPP_ERROR(logger(), "%s: missing required input [target_pose]: %s",
                    name().c_str(), target_pose.error().c_str());
        return false;
    }
    goal.target_pose = target_pose.value();

    auto velocity_override = getInput<int8_t>("velocity_override");
    if (velocity_override) {
        goal.velocity_override = velocity_override.value();
    }

    else {
        goal.velocity_override = 30;  // default velocity
    }

    return true;
}

NodeStatus MoveToAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
    RCLCPP_INFO(logger(), "%s: onResultReceived.Done = %s", name().c_str(),
                wr.result->success ? "true" : "false");

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus MoveToAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
    return NodeStatus::FAILURE;
}

void MoveToAction::onHalt()
{
    RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class MoveToAction will self register with name  "MoveToAction".
CreateRosNodePlugin(MoveToAction, "MoveToAction");