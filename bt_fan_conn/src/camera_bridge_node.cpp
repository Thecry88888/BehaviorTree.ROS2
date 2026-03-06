#include "netinet/in.h"
#include "fcntl.h"
#include "unistd.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <mutex>

#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

using namespace std::chrono_literals;

class CameraBridgeNode : public rclcpp::Node {
public:
    explicit CameraBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("camera_bridge_node", options), serverSocket_(-1), clientSocket_(-1)
    {
        using namespace std::placeholders;
        check_connection_timer_ = this->create_wall_timer(
            1s, std::bind(&CameraBridgeNode::check_socket_connection, this));

        fan_pose_service_ = this->create_service<std_srvs::srv::Trigger>(
            "get_fan_pose",
            std::bind(&CameraBridgeNode::handle_get_fan_pose, this, _1, _2));
        conn_pose_service_ = this->create_service<std_srvs::srv::Trigger>(
            "get_conn_pose",
            std::bind(&CameraBridgeNode::handle_get_conn_pose, this, _1, _2));
    
        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    int serverSocket_;
    int clientSocket_;
    struct sockaddr_in clientAddress_;
    static constexpr int CAMERA_PORT = 10001;
    std::mutex socket_mutex_;

    struct FanPacket{ 
        // task_id, x, y, z, roll ,pitch ,yaw
        float values[7];
    } __attribute__((packed));
    
    rclcpp::TimerBase::SharedPtr check_connection_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fan_pose_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr conn_pose_service_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void handle_get_fan_pose(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        std::string task = "task1";

        if (send(clientSocket_, task.c_str(), task.size(), 0) < 0) { // send task id
            response->success = false;
            response->message = "Socket send failed";
            return;
        }
        
        FanPacket fan_packet;
        std::string error_msg;
        if (!receive_fan_packet(fan_packet, error_msg)) {
            response->success = false;
            response->message = error_msg;
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        // boardcast world_frame to fan_frame tf
        // since we don't have camera to world_frame tf and camera have 2D information only
        t.header.frame_id = "lrmate_200id_world";
        t.child_frame_id = "fan_frame"; 
        t.transform.translation.x = fan_packet.values[1];
        t.transform.translation.y = fan_packet.values[2];
        // t.transform.translation.z = -100; // height need to be verified
        tf2::Quaternion q;
        q.setRPY(
            fan_packet.values[4] * DEG2RAD, // roll = 0
            fan_packet.values[5] * DEG2RAD, // pitch = 0
            fan_packet.values[6] * DEG2RAD  // yaw according to fan
        );
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
        response->success = true;
        response->message = "Fan pose updated via TF";
    }

    void handle_get_conn_pose(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        std::string task = "task2";

        if (send(clientSocket_, task.c_str(), task.size(), 0) < 0) { // send task id
            response->success = false;
            response->message = "Socket send failed";
            return;
        }

        FanPacket fan_packet;
        std::string error_msg;
        if (!receive_fan_packet(fan_packet, error_msg)) {
            response->success = false;
            response->message = error_msg;
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "in_hand_camera_frame";
        t.child_frame_id = "conn_frame";
        t.transform.translation.x = fan_packet.values[1];
        t.transform.translation.y = fan_packet.values[2];
        t.transform.translation.z = fan_packet.values[3];
        // camera can't measure connector's orientation,
        // assume no rotation
        t.transform.rotation.x = 0.0; 
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
        response->success = true;
        response->message = "Fan pose updated via TF";
    }

    bool receive_fan_packet(FanPacket& fan_packet, std::string& error_message) {
        uint8_t* buffer = reinterpret_cast<uint8_t*>(&fan_packet);
        ssize_t total_received = 0;
        ssize_t expected_size = sizeof(FanPacket);

        while (total_received < expected_size) {
            ssize_t bytes_received = recv(clientSocket_, buffer + total_received, expected_size - total_received, 0);
            
            if (bytes_received > 0) {
                total_received += bytes_received;
            } else if (bytes_received == 0) {
                error_message = "Client disconnected";
                return false;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Recv timeout or error!");
                error_message = "Vision PC Timeout";
                return false;
            }
        }
        return true;
    }

    void check_socket_connection() {
        if (serverSocket_ == -1) {
            serverSocket_ = start_socket_server(CAMERA_PORT);
        }

        if (clientSocket_ == -1) {
            clientSocket_ = accept_socket_client();
        }
    }

    int start_socket_server(int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return -1;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(sock, (struct sockaddr*)&address, sizeof(address)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(sock);
            return -1;
        }

        if (listen(sock, 5) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            close(sock);
            return -1;
        }

        // non-blocking mode
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(this->get_logger(), "Server listening on port %d", port);
        return sock;
    }

    int accept_socket_client() {
        socklen_t client_len = sizeof(clientAddress_);
        int client = accept(serverSocket_, (struct sockaddr*)&clientAddress_, &client_len);

        if (client == -1) {
            return -1;  // 尚未有 client 嘗試連線
        }

        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;

        if (setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set timeout on client socket");
        }

        RCLCPP_INFO(this->get_logger(), "Client connected. Timeout set to %2fs", timeout.tv_sec + timeout.tv_usec / 1e6);
        return client;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraBridgeNode>());
    rclcpp::shutdown();
    return 0;
}