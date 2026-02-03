#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/parallel_gripper.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "robot_interfaces/msg/gripper_command.hpp"
#include "robot_interfaces/msg/gripper_info.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define DEG2RAD (M_PI / 180.0)

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
            rclcpp::QoS(10), std::bind(&ParallelGripperServer::update_gripper_state, this, _1));

        this->action_server_ = rclcpp_action::create_server<ParallelGripper>(
            this, "parallel_gripper", std::bind(&ParallelGripperServer::handle_goal, this, _1, _2),
            std::bind(&ParallelGripperServer::handle_cancel, this, _1),
            std::bind(&ParallelGripperServer::handle_accepted, this, _1));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sensor2j6_tf(0.0); // initial position
    }

private:
    rclcpp_action::Server<ParallelGripper>::SharedPtr action_server_;
    rclcpp::Publisher<robot_interfaces::msg::GripperCommand>::SharedPtr gripper_command_publisher_;
    // need motor angle feedback to compute tf
    rclcpp::Subscription<robot_interfaces::msg::GripperInfo>::SharedPtr gripper_info_subscriber_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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

    void update_gripper_state(const robot_interfaces::msg::GripperInfo::SharedPtr msg)
    {
        grip_state_.store(msg->result);
        // manual test command:
        // ros2 topic pub /gripper_info robot_interfaces/msg/GripperInfo "{result: 4, angle: 30.0}" -1
    
        // get motor angle to update tf
        sensor2j6_tf(msg->angle);
        // manual test command:
        // ros2 run tf2_ros tf2_echo link_6 tactile_sensor_frame
    }

    void sensor2j6_tf(float motor_deg) {
        float rad = motor_deg * DEG2RAD;
        float L1 = 34; ///< 曲柄長度
        float L2 = 36.675; ///< 搖桿長度
        float offset = -13.75+3.11; ///< 滑塊偏移量+矽膠凸起
        
        float dz = L1 * cos(rad) + L2 * cos(asin(sin(rad)/L2)) + offset;
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "link_6";
        t.child_frame_id = "tactile_sensor_frame";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = -dz;
        t.transform.translation.z = 194.12;

        // optional: tf2::Quaternion q.setRPY(180, 0, -90);
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.7071;
        t.transform.rotation.z = 0.7071;
        t.transform.rotation.w = 0.0;
        tf_broadcaster_->sendTransform(t);
    }
  
};  // class ParallelGripperServer

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParallelGripperServer>();

    rclcpp::spin(node);
    return 0;
}
