#include "behaviortree_cpp/action_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace BT;

class ComputeTargetPose : public BT::SyncActionNode {
public:
    ComputeTargetPose(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : BT::SyncActionNode(name, config), tf_buffer_(tf_buffer) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_frame"),   // 目標frame (e.g., "fan_frame")
            BT::InputPort<std::string>("tool_frame"),     // 機器人哪部分去 (e.g., "gipper_center_frame")
            BT::InputPort<std::string>("base_frame"),     // 參考座標 (e.g., "base_link")
            BT::InputPort<std::vector<double>>("offset"), // 可選偏移 [x, y, z, r, p, y]
            BT::OutputPort<geometry_msgs::msg::Pose>("target_pose") // 算出的 link_6 座標
        };
    }

    BT::NodeStatus tick() override {
        std::string target, tool, base;
        if (!getInput("target_frame", target) || !getInput("tool_frame", tool) || !getInput("base_frame", base)) {
            return BT::NodeStatus::FAILURE;
        }
        // 在 tick 之前，確認 TF 是否真的連通
        while (rclcpp::ok()) {
            if (tf_buffer_->canTransform(base, target, tf2::TimePointZero)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        try {
            // when tool = target, tool can grip target
            // get T_base_target = T_base_link6 * T_link6_tool
            // -> T_base_link6 = T_base_target * (T_link6_tool)^-1

            // 取得目標在基座下的位置 T_base_target
            auto t_base_target = tf_buffer_->lookupTransform(base, target, tf2::TimePointZero);
            tf2::Transform T_base_target;
            tf2::fromMsg(t_base_target.transform, T_base_target);

            // 取得工具相對於 Flange (link_6) 的固定關係 T_link6_tool
            auto t_link6_tool = tf_buffer_->lookupTransform("link_6", tool, tf2::TimePointZero);
            tf2::Transform T_link6_tool;
            tf2::fromMsg(t_link6_tool.transform, T_link6_tool);

            // T_base_link6 = T_base_target * (T_link6_tool)^-1
            tf2::Transform T_base_link6 = T_base_target * T_link6_tool.inverse();

            // 加上 Approach 偏移，例如在目標上方 5cm (選擇性)
            std::vector<double> offset;
            if (getInput("offset", offset) && offset.size() == 6) {
                tf2::Transform T_offset;
                T_offset.setOrigin(tf2::Vector3(offset[0], offset[1], offset[2]));
                tf2::Quaternion q_off;
                q_off.setRPY(offset[3], offset[4], offset[5]);
                T_offset.setRotation(q_off);
                T_base_link6 = T_base_link6 * T_offset; // 局部座標系偏移
            }

            geometry_msgs::msg::Pose goal;
            tf2::toMsg(T_base_link6, goal);
            setOutput("target_pose", goal);

            return BT::NodeStatus::SUCCESS;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("BT"), "TF 計算失敗: %s", ex.what());
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

static const char* test_xml = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="TestCompute">
        <ComputeTargetPose target_frame="fan_frame" 
                           tool_frame="tactile_sensor_frame"
                            base_frame="base_link"
                           target_pose="{my_test_pose}"/>
    </BehaviorTree>
</root>
)";

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("compute_target_pose_test");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);
    // --- 測試用：廣播一個固定的 TF 來模擬 fan_frame 和 tactile_sensor_frame 的位置關係 ---
    // ros2 run tf2_ros static_transform_publisher -0.0048 0.5 -0.1 0 0 0 base_link fan_frame
    // and
    // ros2 run tf2_ros static_transform_publisher 0.0 -0.005 -0.19412 0 0 0 link_6 tactile_sensor_frame

    BehaviorTreeFactory factory;
    factory.registerBuilder<ComputeTargetPose>(
        "ComputeTargetPose",
        [tf_buffer](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ComputeTargetPose>(name, config, tf_buffer);
        }
    );

    auto tree = factory.createTreeFromText(test_xml);
    NodeStatus status = tree.tickExactlyOnce();

    if (status == BT::NodeStatus::SUCCESS) {
        auto msg = tree.rootBlackboard()->get<geometry_msgs::msg::Pose>("my_test_pose");
        std::cout << "算出的目標 X: " << msg.position.x << " Y: " << msg.position.y << " Z: " << msg.position.z << std::endl;
    } else {
        std::cerr << "節點執行失敗，請檢查 TF 是否存在！" << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}