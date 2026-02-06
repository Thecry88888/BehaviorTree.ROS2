#include "rclcpp/rclcpp.hpp"

#include "behaviortree_ros2/plugins.hpp"

#ifndef USE_TACTILE_SENSOR_PLUGIN
#include "bt_fan_conn/tactile_sensor_action.hpp"
#endif

using namespace BT;

// 沿用您的 PrintValue 節點
class PrintValue : public SyncActionNode {
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
// XML 行為樹
//-------------------------------------------------------------
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <PrintValue message="開始偵測連接器狀態..."/>
            
            <TactileSensorAction name="check_tactile" 
                                 service_name="locate_connector_service"/>

            <PrintValue message="偵測完成，連接器已確認！"/>
        </Sequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("tactile_bt_executor");

    BehaviorTreeFactory factory;

    factory.registerNodeType<PrintValue>("PrintValue");

    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "locate_connector_service"; 

    #ifdef USE_TACTILE_SENSOR_PLUGIN
        RegisterRosNode(factory, "../lib/libtactile_sensor_action_plugin.so", params);
    #else
        factory.registerNodeType<TactileSensorAction>("TactileSensorAction", params);
    #endif

    auto tree = factory.createTreeFromText(xml_text);

    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickExactlyOnce();
        
        // 處理 ROS 2 回調 (Service Response 需要 spin)
        rclcpp::spin_some(nh); 
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(status == NodeStatus::SUCCESS) {
        RCLCPP_INFO(nh->get_logger(), "任務成功結束");
    } else {
        RCLCPP_ERROR(nh->get_logger(), "任務失敗結束");
    }

    rclcpp::shutdown();
    return 0;
}