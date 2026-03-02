#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "moveit_msgs/action/move_group.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/pose.hpp"

#ifndef USE_FOLLOW_PATH_PLUGIN
#include "bt_fan_conn/follow_path_action.hpp"
#endif

using namespace BT;

class PrintValue : public SyncActionNode
{
public:
    PrintValue(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {}

    NodeStatus tick() override {
        std::string msg;
        if(getInput("message", msg)) {
            std::cout << "[BT Log]: " << msg << std::endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    static PortsList providedPorts() {
        return { InputPort<std::string>("message") };
    }
};

//-------------------------------------------------------------
// XML 行為樹定義：給定當前位姿和目標位姿，執行 FollowPathAction
//-------------------------------------------------------------
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <PrintValue message="開始執行任務..."/>
            
            <Timeout msec="50000">
                <FollowPathAction action_name="/move_action" target_pose="{target_pose}"/>
            </Timeout>

            <PrintValue message="全流程完成！"/>
        </Sequence>
    </BehaviorTree>
</root>
)";


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("followpath_bt_executor");

    BehaviorTreeFactory factory;
    factory.registerNodeType<PrintValue>("PrintValue");

    // 註冊 FollowPath 節點
    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "/move_action"; // 這裡是 MoveIt Action Server 的名稱
    
    #ifdef USE_FOLLOW_PATH_PLUGIN
        RegisterRosNode(factory, "../lib/libfollow_path_action_plugin.so", params);
    #else
        factory.registerNodeType<FollowPathAction>("FollowPathAction", params);
    #endif

    auto tree = factory.createTreeFromText(xml_text);

    auto action_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(nh, "/move_action");
    action_client->wait_for_action_server();

    // --- 測試用：在黑板手動設定一個目標位姿 ---
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 0.5; // m (依據您的 Server 設定)
    test_pose.position.y = 0;
    test_pose.position.z = 0.1;
    test_pose.orientation.x = 1;
    test_pose.orientation.y = 0;
    test_pose.orientation.z = 0;
    test_pose.orientation.w = 0;

    tree.rootBlackboard()->set("target_pose", test_pose);
    // 執行迴圈
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickExactlyOnce();
        rclcpp::spin_some(nh); 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
