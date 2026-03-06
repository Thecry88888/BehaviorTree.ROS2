#include "bt_fan_conn/follow_path_action.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

bool FollowPathAction::setGoal(RosActionNode::Goal& goal)
{
    auto target_pose = getInput<geometry_msgs::msg::Pose>("target_pose");
    if (!target_pose) return false;

    moveit_msgs::msg::Constraints constr;
    constr.name = "goal_constraints";

    // 位置約束 (Position Constraint)
    moveit_msgs::msg::PositionConstraint pos_con;
    pos_con.header.frame_id = "lrmate_200id_world";
    pos_con.link_name = "link_6";
    
    // 定義一個微小的目標區域
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.02, 0.02, 0.02}; // 2cm 容差
    
    pos_con.constraint_region.primitives.push_back(box);
    pos_con.constraint_region.primitive_poses.push_back(target_pose.value());
    pos_con.weight = 1.0;

    constr.position_constraints.push_back(pos_con);

    // 姿勢約束 (Orientation Constraint)
    moveit_msgs::msg::OrientationConstraint ori_con;
    ori_con.header.frame_id = "lrmate_200id_world";
    ori_con.link_name = "link_6";
    ori_con.orientation = target_pose.value().orientation;
    ori_con.absolute_x_axis_tolerance = 0.0175; // 弧度容差
    ori_con.absolute_y_axis_tolerance = 0.0175;
    ori_con.absolute_z_axis_tolerance = 0.0175;
    ori_con.weight = 1.0;
    
    constr.orientation_constraints.push_back(ori_con);

    goal.request.group_name = "lrmate_200id"; // 規劃組名稱
    goal.request.pipeline_id = "pilz_industrial_motion_planner";
    // 指定規劃器運動模式
    // LIN: 笛卡兒直線運動 (TCP 走直線，姿態平滑 SLERP)
    // PTP: 點對點關節運動
    goal.request.planner_id = "PTP";
    goal.request.num_planning_attempts = 1; // Pilz is deterministic
    goal.request.allowed_planning_time = 2.0;
    goal.request.start_state.is_diff = true; // 從當前狀態開始規劃
    goal.request.goal_constraints.push_back(constr);

    return true;
}

NodeStatus FollowPathAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
    if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(logger(), "Action failure, error code: %d", static_cast<int>(wr.code));
        return NodeStatus::FAILURE;
    }

    auto error_code = wr.result->error_code.val;

    using Error = moveit_msgs::msg::MoveItErrorCodes;

    switch (error_code) {
        case Error::SUCCESS:
            RCLCPP_INFO(logger(), "MoveGroup: success");
            return NodeStatus::SUCCESS;

        case Error::PLANNING_FAILED:
            RCLCPP_ERROR(logger(), "MoveGroup: planning failed");
            return NodeStatus::FAILURE;

        case Error::CONTROL_FAILED:
        // This usually occurs when the Fanuc Node (underlying controller) reports a failure, and MoveIt forwards it.
            RCLCPP_ERROR(logger(), "MoveGroup: control failed");
            return NodeStatus::FAILURE;

        case Error::TIMED_OUT:
            RCLCPP_ERROR(logger(), "MoveGroup: timed out");
            return NodeStatus::FAILURE;

        case Error::INVALID_MOTION_PLAN:
            RCLCPP_ERROR(logger(), "MoveGroup: invalid motion plan");
            return NodeStatus::FAILURE;

        default:
            RCLCPP_ERROR(logger(), "MoveGroup: unknown error, code: %d", error_code);
            return NodeStatus::FAILURE;
    }
}

NodeStatus FollowPathAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
    return NodeStatus::FAILURE;
}

void FollowPathAction::onHalt()
{
    RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
    RosActionNode<moveit_msgs::action::MoveGroup>::onHalt();
}

// Plugin registration.
// The class FollowPathAction will self register with name  "FollowPathAction".
CreateRosNodePlugin(FollowPathAction, "FollowPathAction");