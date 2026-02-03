#include "behaviortree_ros2/plugins.hpp"

using namespace BT;

class IsConnectorPlugged : public ConditionNode {
public:
    IsConnectorPlugged(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<bool>("plug_status") };
    }

    NodeStatus tick() override {
        auto res = getInput<bool>("plug_status");
        
        if (!res) {
            return NodeStatus::FAILURE;
        }

        return res.value() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};