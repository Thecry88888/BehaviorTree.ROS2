#include "behaviortree_cpp/action_node.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace BT;

class ComputeTargetPose : public SyncActionNode
{
public:
    ComputeTargetPose(const std::string& name, const NodeConfig& config, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : SyncActionNode(name, config), tf_buffer_(tf_buffer) {}

    static PortsList providedPorts() {
        return { 
            InputPort<geometry_msgs::msg::Pose>("object_goal"), // 連接器最終要去的位置
            OutputPort<geometry_msgs::msg::Pose>("link6_target")    // 手臂應該去的位置
        };
    }

    NodeStatus tick() override {
        auto goal_res = getInput<geometry_msgs::msg::Pose>("object_goal");
        if (!goal_res) return NodeStatus::FAILURE;

        try {
            // 1. 取得當前 link_6 到 connector_frame 的相對關係 (這段是固定的)
            auto t_link6_to_conn = tf_buffer_->lookupTransform("link_6", "connector_frame", tf2::TimePointZero);
            
            tf2::Transform tf2_link6_to_conn;
            tf2::fromMsg(t_link6_to_conn.transform, tf2_link6_to_conn);

            // 2. 取得目標位置 T_base_to_goal
            tf2::Transform tf2_base_to_goal;
            tf2::fromMsg(goal_res.value(), tf2_base_to_goal);

            // 3. 計算手臂目標：T_base_to_link6 = T_base_to_goal * (T_link6_to_conn).inverse()
            tf2::Transform tf2_base_to_link6 = tf2_base_to_goal * tf2_link6_to_conn.inverse();

            // 4. 輸出給黑板
            geometry_msgs::msg::Pose link6_target;
            tf2::toMsg(tf2_base_to_link6, link6_target);
            setOutput("link6_target", link6_target);

            return NodeStatus::SUCCESS;
        } catch (tf2::TransformException &ex) {
            return NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};