#include "netinet/in.h"
#include "fcntl.h"
#include "unistd.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "bt_fan_conn/command_encode.hpp"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

using namespace FANUC::CommandEncode;
using namespace std::chrono_literals;

class FanucBridgeNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit FanucBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("fanuc_bridge_node", options), serverSocket_(-1), clientSocket_(-1)
    {   
        using namespace std::placeholders;

        // initialize after construction (shared_from_this() is safe in callback)
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]() {
                init_timer_->cancel();
                init_moveit();
            });

        check_connection_timer_ = this->create_wall_timer(
            1s, std::bind(&FanucBridgeNode::check_socket_connection, this));

        euler_state_timer_ = this->create_wall_timer(
            20ms, std::bind(&FanucBridgeNode::get_euler_state, this));

        joint_state_timer_ = this->create_wall_timer(
            20ms, std::bind(&FanucBridgeNode::get_joint_state, this));
        
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        cmd_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "cmd_pose", 10, 
            std::bind(&FanucBridgeNode::handle_cmd_pose, this, _1));

        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "lrmate_200id_controller/follow_joint_trajectory", // 必須與 moveit_controllers.yaml 一致
            std::bind(&FanucBridgeNode::handle_goal, this, _1, _2),
            std::bind(&FanucBridgeNode::handle_cancel, this, _1),
            std::bind(&FanucBridgeNode::handle_accepted, this, _1));

        tf_broadcaster_ =
            std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    int serverSocket_;
    int clientSocket_;
    struct sockaddr_in clientAddress_;
    static constexpr int FANUC_PORT = 12000;
    std::array<float, 6> current_euler_states_; // x, y, z, roll(w), pitch(p), yaw(r)

    struct CommandHeader {
        int robot_num;
        int cmd_id;
    } __attribute__((packed));

    rclcpp::TimerBase::SharedPtr check_connection_timer_;
    rclcpp::TimerBase::SharedPtr euler_state_timer_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscriber_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_{nullptr};

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal> goal) 
    {
        RCLCPP_INFO(this->get_logger(), "收到新軌跡，點數: %zu", goal->trajectory.points.size());
        if (goal->trajectory.points.empty()) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
        RCLCPP_INFO(this->get_logger(), "收到取消請求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{std::bind(&FanucBridgeNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        
        const float TOLERANCE = 1.0f; 

        for (const auto& point : goal->trajectory.points) {
            if (goal_handle->is_canceling()) {
                result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "軌跡執行已取消");
                return;
            }

            kinematic_state_->setJointGroupPositions(joint_model_group_, point.positions);

            // FK 取得 link_6 to base_link 的 Pose
            const Eigen::Isometry3d& end_effector_state = kinematic_state_->getGlobalLinkTransform("link_6");
            std::array<float, 6> target_euler_pose;
            target_euler_pose[0] = static_cast<float>(end_effector_state.translation().x());
            target_euler_pose[1] = static_cast<float>(end_effector_state.translation().y());
            target_euler_pose[2] = static_cast<float>(end_effector_state.translation().z());

            Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(2, 1, 0); // Z-Y-X intrinsic order
            target_euler_pose[3] = static_cast<float>(euler_angles[2] * RAD2DEG);
            target_euler_pose[4] = static_cast<float>(euler_angles[1] * RAD2DEG);
            target_euler_pose[5] = static_cast<float>(euler_angles[0] * RAD2DEG);

            move(target_euler_pose);

            // 等待抵達目標點位
            bool reached = false;
            auto start_time = this->now();
            
            while (!reached && rclcpp::ok()) {
                bool all_in_range = true;
                for (size_t i = 0; i < 6; ++i) {
                    if (std::abs(target_euler_pose[i] - current_euler_states_[i]) > TOLERANCE) {
                        all_in_range = false;
                        break;
                    }
                }

                if (all_in_range) {
                    reached = true;
                } else {
                    rclcpp::sleep_for(std::chrono::milliseconds(100)); 
                }
            }
        }
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "軌跡執行成功完成");
    }

    void init_moveit() {
        robot_model_loader_ =
            std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this());
        kinematic_model_ = robot_model_loader_->getModel();
        kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        joint_model_group_ = kinematic_model_->getJointModelGroup("lrmate_200id");
    }

    void handle_cmd_pose(const geometry_msgs::msg::Pose::SharedPtr msg) {
        double w, p, r;
        q_to_wpr(msg->orientation, w, p, r);
        std::array<float, 6> target_pose = {
            static_cast<float>(msg->position.x),
            static_cast<float>(msg->position.y),
            static_cast<float>(msg->position.z),
            static_cast<float>(w),
            static_cast<float>(p),
            static_cast<float>(r)
        };
        move(target_pose);
    }

    void q_to_wpr(const geometry_msgs::msg::Quaternion& q, double& w, double& p, double& r) {
        tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf2_q);
        m.getRPY(w, p, r);
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
        recv(clientSocket_, current_euler_states_.data(), sizeof(current_euler_states_), 0);
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

    void move(const std::array<float, 6>& euler_pose) {
        CommandHeader header = {0, CMD_MOVE};
        send(clientSocket_, &header, sizeof(header), 0);
        send(clientSocket_, euler_pose.data(), sizeof(euler_pose), 0);

        // script 指令
        send_script(0);
    }

    void move_joint(const std::array<float, 6>& joint_angles) {
        CommandHeader header = {0, CMD_MOVE_JOINT};
        send(clientSocket_, &header, sizeof(header), 0);
        send(clientSocket_, joint_angles.data(), sizeof(joint_angles), 0);
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