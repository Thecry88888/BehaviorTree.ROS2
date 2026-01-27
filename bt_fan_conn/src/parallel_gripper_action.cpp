#include "bt_fan_conn/parallel_gripper_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool ParallelGripperAction::setGoal(RosActionNode::Goal& goal)
{
  auto command = getInput<int8_t>("command");
  goal.command = command.value();
  return true;
}

NodeStatus ParallelGripperAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived.Done = %s", name().c_str(),
              wr.result->success ? "true" : "false");

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus ParallelGripperAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void ParallelGripperAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class ParallelGripperAction will self register with name  "ParallelGripperAction".
CreateRosNodePlugin(ParallelGripperAction, "ParallelGripperAction");