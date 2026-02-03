#include "behaviortree_ros2/plugins.hpp"

using namespace BT;

class IsFanPlaced : public ConditionNode {
public:
    IsFanPlaced(const std::string& name, const NodeConfig& config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts() {
        return { InputPort<bool>("fan_state") };
    }

    NodeStatus tick() override {
        auto res = getInput<bool>("fan_state");
        
        if (!res) {
            return NodeStatus::FAILURE;
        }

        return res.value() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};