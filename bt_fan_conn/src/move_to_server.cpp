#include <memory>
#include <string>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/move_to.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class MoveToServer : public rclcpp::Node {
public:
    using MoveTo = btcpp_ros2_interfaces::action::MoveTo;
    using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

    explicit MoveToServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("move_to_server", options)
    {
        using namespace std::placeholders;

        cmd_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("cmd_pose", 10);
        
        this->action_server_ = rclcpp_action::create_server<MoveTo>(
            this, "move_to",
            std::bind(&MoveToServer::handle_goal, this, _1, _2),
            std::bind(&MoveToServer::handle_cancel, this, _1),
            std::bind(&MoveToServer::handle_accepted, this, _1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_publisher_;
    rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Action Boilerplate
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveTo::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTo>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
        std::thread{std::bind(&MoveToServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveTo::Result>();
        auto feedback = std::make_shared<MoveTo::Feedback>();

        tf2::Vector3 target_pos(
            goal->target_pose.position.x,
            goal->target_pose.position.y,
            goal->target_pose.position.z);

        tf2::Quaternion target_q;
        tf2::fromMsg(goal->target_pose.orientation, target_q);
        const float pos_tolerance = 0.001;   // meter
        const float ang_tolerance = 0.017;   // rad
        float dist = std::numeric_limits<float>::max();
        float angle_diff = std::numeric_limits<float>::max();

         // 發佈目標位姿到 cmd_pose 主題
        cmd_pose_publisher_->publish(goal->target_pose);
        
        rclcpp::Rate loop_rate(1); // 1Hz 監控頻率

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            geometry_msgs::msg::TransformStamped t;
            try {
                t = tf_buffer_->lookupTransform(
                    "lrmate_200id_world", "tool0", tf2::TimePointZero); // newest
                // manual test command:
                // ros2 run tf2_ros static_transform_publisher --x 100 --y 50 --z 200 --yaw 0 --pitch 0 --roll 0 --frame-id lrmate_200id_world --child-frame-id tool0
                // same as move_to_client test pose
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                loop_rate.sleep();
                continue;
            }

            tf2::Vector3 current_pos(
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z);

            tf2::Quaternion current_q;
            tf2::fromMsg(t.transform.rotation, current_q);

            // 歐幾里得距離
            dist = target_pos.distance(current_pos);

            // 角度誤差
            angle_diff = current_q.angle(target_q);

            feedback->current_pose.position.x = t.transform.translation.x;
            feedback->current_pose.position.y = t.transform.translation.y;
            feedback->current_pose.position.z = t.transform.translation.z;
            feedback->current_pose.orientation = t.transform.rotation;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Target Reached (Dist: %.3f m)", dist);
            // 判斷是否抵達
            if (dist < pos_tolerance && angle_diff < ang_tolerance) {
                result->success = true;
                goal_handle->succeed(result);
                return;
            }

            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveToServer>());
    rclcpp::shutdown();
    return 0;
}