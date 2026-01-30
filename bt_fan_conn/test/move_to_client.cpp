#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/pose.hpp"

#ifndef USE_MOVE_TO_PLUGIN
#include "bt_fan_conn/move_to_action.hpp"
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
// XML 行為樹定義：包含 MoveTo 與 Gripper 的順序組合
//-------------------------------------------------------------
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <PrintValue message="開始移動任務..."/>
            
            <Timeout msec="10000">
                <MoveToAction action_name="move_to" target_pose="{my_pose}"/>
            </Timeout>

            <PrintValue message="全流程完成！"/>
        </Sequence>
    </BehaviorTree>
</root>
)";


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("moveto_bt_executor");

    BehaviorTreeFactory factory;
    factory.registerNodeType<PrintValue>("PrintValue");

    // 註冊 MoveTo 節點
    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "move_to_service";
    
    #ifdef USE_MOVE_TO_PLUGIN
        RegisterRosNode(factory, "../lib/libmove_to_action_plugin.so", params);
    #else
        factory.registerNodeType<MoveToAction>("MoveToAction", params);
    #endif

    auto tree = factory.createTreeFromText(xml_text);

    // --- 測試用：在黑板手動設定一個目標位姿 ---
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 100.0; // mm (依據您的 Server 設定)
    test_pose.position.y = 50.0;
    test_pose.position.z = 200.0;
    test_pose.orientation.w = 1.0; 
    tree.rootBlackboard()->set("my_pose", test_pose);

    // 執行迴圈
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickExactlyOnce();
        rclcpp::spin_some(nh); 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}