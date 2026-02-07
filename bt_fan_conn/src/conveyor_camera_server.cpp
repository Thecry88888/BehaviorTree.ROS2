#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/locate_object.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

class ConveyorCameraServer : public rclcpp::Node{
public:
    using LocateObject = btcpp_ros2_interfaces::action::LocateObject;
    using GoalHandleLocateObject = rclcpp_action::ServerGoalHandle<LocateObject>;
    explicit ConveyorCameraServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("conveyor_camera_server", options)
    {   
        fan_pose_client_ = this->create_client<std_srvs::srv::Trigger>(
            "get_fan_pose_from_camera");

        this->action_server_ = rclcpp_action::create_server<LocateObject>(
            this,
            "locate_fan_action",
            std::bind(&ConveyorCameraServer::handle_goal, this, _1, _2),
            std::bind(&ConveyorCameraServer::handle_cancel, this, _1),
            std::bind(&ConveyorCameraServer::handle_accepted, this, _1));
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr fan_pose_client_;
    rclcpp_action::Server<LocateObject>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const LocateObject::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLocateObject>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLocateObject> goal_handle) {
        std::thread{std::bind(&ConveyorCameraServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLocateObject> goal_handle) {
        auto result = std::make_shared<LocateObject::Result>();

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); // call camera bridge node
        
        while (!fan_pose_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok() || goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待 Camera Bridge Service...");
        }

        auto future = fan_pose_client_->async_send_request(request);
        
        // timeout after 5 seconds
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) { 
            auto srv_res = future.get();
            result->success = srv_res->success;
            result->message = srv_res->message;
        } else {
            result->success = false;
            result->message = "Camera Bridge Timeout";
        }

        goal_handle->succeed(result);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorCameraServer>());
    rclcpp::shutdown();
    return 0;
}