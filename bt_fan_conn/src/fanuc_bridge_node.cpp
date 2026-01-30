#include "netinet/in.h"
#include "fcntl.h"
#include "unistd.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "bt_fan_conn/command_encode.hpp"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

using namespace FANUC::CommandEncode;
using namespace std::chrono_literals;

class FanucBridgeNode : public rclcpp::Node {
public:
    explicit FanucBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("fanuc_bridge_node", options), serverSocket_(-1), clientSocket_(-1)
    {   
        using namespace std::placeholders;
        check_connection_timer_ = this->create_wall_timer(
            1s, std::bind(&FanucBridgeNode::check_socket_connection, this));
        
        euler_state_timer_ = this->create_wall_timer(
            100ms, std::bind(&FanucBridgeNode::get_euler_state, this));

        joint_state_timer_ = this->create_wall_timer(
            100ms, std::bind(&FanucBridgeNode::get_joint_state, this));
        
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        cmd_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "cmd_pose", 10, 
            std::bind(&FanucBridgeNode::handle_cmd_pose, this, _1));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    int serverSocket_;
    int clientSocket_;
    struct sockaddr_in clientAddress_;
    static constexpr int FANUC_PORT = 12000;

    struct CommandHeader {
        int robot_num;
        int cmd_id;
    } __attribute__((packed));

    rclcpp::TimerBase::SharedPtr check_connection_timer_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr euler_state_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscriber_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void handle_cmd_pose(const geometry_msgs::msg::Pose::SharedPtr msg) {
        double w, p, r;
        q_to_wpr(msg->orientation, w, p, r);
        move(msg->position.x, msg->position.y, msg->position.z, w, p, r);
    }

    void q_to_wpr(const geometry_msgs::msg::Quaternion& q, double& w, double& p, double& r) {
        tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf2_q);
        m.getRPY(r, p, w); // 注意：tf2 的 RPY 對應 FANUC 可能需要調整順序
        w *= RAD2DEG; p *= RAD2DEG; r *= RAD2DEG;
    }

    void set_override(int8_t value) {
        CommandHeader header = {0, CMD_SET_OVERRIDE};
        send(clientSocket_, &header, sizeof(header), 0);

        int override = static_cast<int>(value);
        send(clientSocket_, &override, sizeof(override), 0);
    }

    void get_euler_state() {
        CommandHeader header = {0, CMD_GET_EULER_INFO};
        send(clientSocket_, &header, sizeof(header), 0);

        float values[6];
        recv(clientSocket_, values, sizeof(values), 0);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "link_6"; // parameterize?
        t.transform.translation.x = values[0];
        t.transform.translation.y = values[1];
        t.transform.translation.z = values[2];

        tf2::Quaternion q;
        // Z-Y-X intrinsic order
        // euler[3]：roll（X 軸）euler[4]：pitch（Y 軸）euler[5]：yaw（Z 軸）
        q.setRPY(
            values[3] * DEG2RAD, // W (X-roll)
            values[4] * DEG2RAD, // P (Y-pitch)
            values[5] * DEG2RAD  // R (Z-yaw)
        );
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    void get_joint_state() {
        CommandHeader header = {0, CMD_GET_JOINT_INFO};
        send(clientSocket_, &header, sizeof(header), 0);

        float values[6];
        recv(clientSocket_, values, sizeof(values), 0);

        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        };
        joint_state_msg.position.resize(6);
        for (size_t i = 0; i < 6; ++i) {
            joint_state_msg.position[i] = values[i] * DEG2RAD;
        }

        joint_state_publisher_->publish(joint_state_msg);
    }

    void move(float x, float y, float z, float w, float p, float r) {
        CommandHeader header = {0, CMD_MOVE};
        send(clientSocket_, &header, sizeof(header), 0);
        float param[6] = {x, y, z, w, p, r};
        send(clientSocket_, param, sizeof(param), 0);

        // script 指令
        send_script(0);
    }

    void send_script(int script_id) {
        CommandHeader header = {0, CMD_SEND_SCRIPT};
        send(clientSocket_, &header, sizeof(header), 0);

        int16_t script = static_cast<int16_t>(script_id);
        send(clientSocket_, &script, sizeof(script), 0);
    }

    void check_socket_connection() {
        if (serverSocket_ == -1) {
            serverSocket_ = start_socket_server(FANUC_PORT);
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

        RCLCPP_INFO(this->get_logger(), "Server listening on port %d", port);
        return sock;
    }

    int accept_socket_client() {
        socklen_t client_len = sizeof(clientAddress_);
        int client = accept(serverSocket_, (struct sockaddr*)&clientAddress_, &client_len);

        if (client == -1) {
            return -1;  // 尚未有 client 嘗試連線
        }

        RCLCPP_INFO(this->get_logger(), "Client connected.");
        return client;
    }

};    

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FanucBridgeNode>());
    rclcpp::shutdown();
    return 0;
}