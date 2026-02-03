#include "behaviortree_ros2/plugins.hpp"

using namespace BT;

class IsConnectorGripped : public ConditionNode {
public:
    IsConnectorGripped(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<bool>("grip_state") };
    }

    NodeStatus tick() override {
        auto res = getInput<bool>("grip_state");
        
        if (!res) {
            return NodeStatus::FAILURE;
        }

        return res.value() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};