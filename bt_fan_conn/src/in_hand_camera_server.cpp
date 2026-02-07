#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "btcpp_ros2_interfaces/srv/locate_object.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class InHandCameraServer : public rclcpp::Node{
public:
    explicit InHandCameraServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("in_hand_camera_server", options)
    {   
        using namespace std::placeholders;
        conn_pose_client_ = this->create_client<std_srvs::srv::Trigger>(
            "get_conn_pose");

        service_ = this->create_service<btcpp_ros2_interfaces::srv::LocateObject>(
            "locate_connector_service",
            std::bind(&InHandCameraServer::handle_service, this, _1, _2));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
        camera_to_j6_tf();
    }


private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr conn_pose_client_;
    rclcpp::Service<btcpp_ros2_interfaces::srv::LocateObject>::SharedPtr service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void handle_service(const std::shared_ptr<btcpp_ros2_interfaces::srv::LocateObject::Request> request,
                        std::shared_ptr<btcpp_ros2_interfaces::srv::LocateObject::Response> response) 
    {
        // call camera service to get connector pose
        auto result = conn_pose_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto res = result.get();
            if (res->success) {
                RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
                response->success = true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get connector pose from camera: %s", res->message.c_str());
                response->success = false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call to camera node failed");
            response->success = false;
        }
    }

    void camera_to_j6_tf() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "link_6";
        t.child_frame_id = "in_hand_camera_frame";
        t.transform.translation.x = 66;
        t.transform.translation.y = 40.25;
        t.transform.translation.z = 125.47;

        tf2::Quaternion q;
        q.setRPY(0, 0, 90 * M_PI / 180.0);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InHandCameraServer>());
    rclcpp::shutdown();
    return 0;
}