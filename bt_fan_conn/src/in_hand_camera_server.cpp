#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/locate_object.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::placeholders;

class InHandCameraServer : public rclcpp::Node{
public:
    using LocateObject = btcpp_ros2_interfaces::action::LocateObject;
    using GoalHandleLocateObject = rclcpp_action::ServerGoalHandle<LocateObject>;
    explicit InHandCameraServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("in_hand_camera_server", options)
    {   
        conn_pose_client_ = this->create_client<std_srvs::srv::Trigger>(
            "get_conn_pose");

        // maunal test
        // ros2 action send_goal locate_conn_action btcpp_ros2_interfaces/action/LocateObject "{start_detection: true}"
        this->action_server_ = rclcpp_action::create_server<LocateObject>(
            this,
            "locate_conn_action",
            std::bind(&InHandCameraServer::handle_goal, this, _1, _2),
            std::bind(&InHandCameraServer::handle_cancel, this, _1),
            std::bind(&InHandCameraServer::handle_accepted, this, _1));

        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    
        camera_to_j6_tf(); // Publish static transforms once at startup
    }


private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr conn_pose_client_;
    rclcpp_action::Server<LocateObject>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const LocateObject::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLocateObject>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLocateObject> goal_handle) {
        std::thread{std::bind(&InHandCameraServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLocateObject> goal_handle) {
        auto result = std::make_shared<LocateObject::Result>();

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); // call camera bridge node
        
        while (!conn_pose_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok() || goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待 Camera Bridge Service...");
        }

        auto future = conn_pose_client_->async_send_request(request);
        
        // timeout after 5 seconds
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) { 
            auto srv_res = future.get();
            result->success = srv_res->success;
            result->message = srv_res->message;
        } else {
            result->success = false;
            result->message = "Camera Bridge Service Timeout";
        }
        
        goal_handle->succeed(result);
    }

    void camera_to_j6_tf() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "tool0";
        t.child_frame_id = "in_hand_camera_frame";
        t.transform.translation.x = 0.066; // meter
        t.transform.translation.y = 0.04025;
        t.transform.translation.z = 0.12547;

        tf2::Quaternion q;
        q.setRPY(0, 0, 90 * M_PI / 180.0);
        t.transform.rotation = tf2::toMsg(q);
        tf_static_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InHandCameraServer>());
    rclcpp::shutdown();
    return 0;
}