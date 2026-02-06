#include "rclcpp/rclcpp.hpp"
#include "btcpp_ros2_interfaces/srv/locate_object.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TactileSensorServer : public rclcpp::Node {
public:
    TactileSensorServer() : Node("sensor_locate_server") {
        using namespace std::placeholders;

        // optional: service get the latest connector pose
        conn_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "connector_pose", rclcpp::QoS(10), std::bind(&TactileSensorServer::connector_pose_callback, this, _1));

        service_ = this->create_service<btcpp_ros2_interfaces::srv::LocateObject>(
            "locate_connector_service",
            std::bind(&TactileSensorServer::handle_service, this, _1, _2));
        
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr conn_pose_subscriber_;
    rclcpp::Service<btcpp_ros2_interfaces::srv::LocateObject>::SharedPtr service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void connector_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        // 2D information only
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "tactile_sensor_frame";
        t.child_frame_id = "connector_frame";
        t.transform.translation.x = msg->position.x;
        t.transform.translation.y = msg->position.y;
        t.transform.rotation = msg->orientation;
        tf_broadcaster_->sendTransform(t);
    }

    void handle_service(const std::shared_ptr<btcpp_ros2_interfaces::srv::LocateObject::Request> request,
                        std::shared_ptr<btcpp_ros2_interfaces::srv::LocateObject::Response> response) 
    {
        // check connector is gripped by sensor?
        response->success = true;

        // failure case
        // RCLCPP_WARN(this->get_logger(), "grip connector failed!");
        // response->success = false;
        return;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TactileSensorServer>());
    rclcpp::shutdown();
    return 0;
}