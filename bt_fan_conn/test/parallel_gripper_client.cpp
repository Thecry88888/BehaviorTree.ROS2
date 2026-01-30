#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_ros2/plugins.hpp"

#ifndef USE_PARALLEL_GRIPPER_PLUGIN
#include "bt_fan_conn/parallel_gripper_action.hpp"
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
// XML 行為樹定義
//-------------------------------------------------------------
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <PrintValue message="準備執行夾取動作..."/>
            
            <Timeout msec="5000">
                <ParallelGripperAction action_name="parallel_gripper" command="1"/>
            </Timeout>

            <PrintValue message="夾取程序完成！"/>
        </Sequence>
    </BehaviorTree>
</root>
)";


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("parallel_gripper_bt_executor");

    BehaviorTreeFactory factory;

    // 註冊普通節點
    factory.registerNodeType<PrintValue>("PrintValue");

    // 註冊 ROS 2 Action 節點
    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "parallel_gripper_service";

    #ifdef USE_PARALLEL_GRIPPER_PLUGIN
        RegisterRosNode(factory, "../lib/libparallel_gripper_action_plugin.so", params);
    #else
        factory.registerNodeType<ParallelGripperAction>("ParallelGripperAction", params);
    #endif

    auto tree = factory.createTreeFromText(xml_text);

    // 執行迴圈：直到行為樹結束為止
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickExactlyOnce();
        
        // 必須處理 ROS 2 的回調，Action 才能運作
        rclcpp::spin_some(nh); 
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}