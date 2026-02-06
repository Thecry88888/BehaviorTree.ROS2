#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "btcpp_ros2_interfaces/srv/locate_object.hpp"


class ConveyorCameraServer : public rclcpp::Node{
public:
    explicit ConveyorCameraServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("conveyor_camera_server", options)
    {   
        using namespace std::placeholders;
        fan_pose_client_ = this->create_client<std_srvs::srv::Trigger>(
            "get_fan_pose_from_camera");

        service_ = this->create_service<btcpp_ros2_interfaces::srv::LocateObject>(
            "locate_fan_service",
            std::bind(&ConveyorCameraServer::handle_service, this, _1, _2));
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr fan_pose_client_;
    rclcpp::Service<btcpp_ros2_interfaces::srv::LocateObject>::SharedPtr service_;

    void handle_service(const std::shared_ptr<btcpp_ros2_interfaces::srv::LocateObject::Request> request,
                        std::shared_ptr<btcpp_ros2_interfaces::srv::LocateObject::Response> response) 
    {
        // call camera service to get fan pose
        auto result = fan_pose_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto res = result.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
                response->success = true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get fan pose from camera: %s", res->message.c_str());
                response->success = false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call to camera node failed");
            response->success = false;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConveyorCameraServer>());
    rclcpp::shutdown();
    return 0;
}