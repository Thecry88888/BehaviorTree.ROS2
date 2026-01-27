#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/parallel_gripper.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "robot_interfaces/msg/gripper_command.hpp"
#include "robot_interfaces/msg/gripper_info.hpp"


class ParallelGripperServer : public rclcpp::Node
{
public:
    using ParallelGripper = btcpp_ros2_interfaces::action::ParallelGripper;
    using GoalHandleParallelGripper = rclcpp_action::ServerGoalHandle<ParallelGripper>;
    explicit ParallelGripperServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("parallel_gripper_server", options)
    {
        using namespace std::placeholders;

        this->gripper_command_publisher_ = this->create_publisher<robot_interfaces::msg::GripperCommand>("gripper_command", 10);

        this->gripper_info_subscriber_ = this->create_subscription<robot_interfaces::msg::GripperInfo>("gripper_info", 
            rclcpp::QoS(10), std::bind(&ParallelGripperServer::update_gripper_info, this, _1));

        this->action_server_ = rclcpp_action::create_server<ParallelGripper>(
            this, "parallel_gripper", std::bind(&ParallelGripperServer::handle_goal, this, _1, _2),
            std::bind(&ParallelGripperServer::handle_cancel, this, _1),
            std::bind(&ParallelGripperServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<ParallelGripper>::SharedPtr action_server_;
    rclcpp::Publisher<robot_interfaces::msg::GripperCommand>::SharedPtr gripper_command_publisher_;
    rclcpp::Subscription<robot_interfaces::msg::GripperInfo>::SharedPtr gripper_info_subscriber_;

    std::atomic<int> grip_state_ = 0; ///< 4:GRIP_SUCCESS, 5:GRIP_MISSED

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                            std::shared_ptr<const ParallelGripper::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with command %d",
                    goal->command);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleParallelGripper> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleParallelGripper> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{ std::bind(&ParallelGripperServer::execute, this, _1), goal_handle }.detach();
    }

    void execute(const std::shared_ptr<GoalHandleParallelGripper> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing Gripper Action Execution");
        this->grip_state_ = 0;

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ParallelGripper::Feedback>();
        auto result = std::make_shared<ParallelGripper::Result>();

        gripper_action(goal->command);
        
        rclcpp::Rate loop_rate(1);  // 1 Hz
        rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds(10);
        while(rclcpp::ok() && get_clock()->now() < deadline)
        {
            if(goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            int current_state = grip_state_.load();

            if (current_state == 4 || current_state == 6) // GRIP_SUCCESS or PUT_SUCCESS
            {
                result->success = true;
                result->grip_state = current_state;
                goal_handle->succeed(result);
                return;
            }
            else if (current_state == 5) // GRIP_MISSED
            {
                result->success = false;
                result->grip_state = current_state;
                goal_handle->succeed(result);
                return;
            }

            loop_rate.sleep();
        }

        // 3. 超時處理
        if (rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Grip Action Timeout!");
            result->success = false;
            result->grip_state = 0;
            goal_handle->abort(result);
        }
        return;
    }

    void gripper_action(int gripperCommand)
    {
        robot_interfaces::msg::GripperCommand msg;
        msg.num = gripperCommand;
        gripper_command_publisher_->publish(msg);
    }

    void update_gripper_info(const robot_interfaces::msg::GripperInfo::SharedPtr msg)
    {
        grip_state_.store(msg->result);
    }
  
};  // class ParallelGripperServer

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParallelGripperServer>();

    rclcpp::spin(node);
    return 0;
}
