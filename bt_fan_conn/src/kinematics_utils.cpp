#include "bt_fan_conn/kinematics_utils.hpp"

using CartesianPose = KinematicsUtils::CartesianPose;
using TrajectoryPoint = KinematicsUtils::TrajectoryPoint;

KinematicsUtils::KinematicsUtils(const rclcpp::Node::SharedPtr& node,
    const std::string& group_name, const std::string& base_frame, const std::string& tip_frame)
    : planning_group_(group_name), base_frame_(base_frame), tip_frame_(tip_frame) 
{
    // 使用 RobotModelLoader 加載機器人模型
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    kinematic_model_ = robot_model_loader.getModel();
    if (!kinematic_model_) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load robot model");
        throw std::runtime_error("Failed to load robot model");
    }

    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_);
}

std::vector<CartesianPose> KinematicsUtils::simplifyTrajectory(
    const std::vector<TrajectoryPoint>& points, 
    double epsilon) 
{   
    // 預計算所有點的 FK
    std::vector<CartesianPose> all_poses;
    for (const auto& point : points) {
        all_poses.push_back(computeFK(point.positions));
    }

    if (points.size() <= 2) return all_poses;

    std::vector<bool> keep(all_poses.size(), false);
    keep.front() = true;
    keep.back() = true;

    rdpRecursive(all_poses, epsilon, 0, all_poses.size() - 1, keep);

    std::vector<CartesianPose> simplified;
    for (size_t i = 0; i < all_poses.size(); ++i) {
        if (keep[i]) simplified.push_back(all_poses[i]);
    }

    return simplified;
}

void KinematicsUtils::rdpRecursive(const std::vector<CartesianPose>& points, 
    double epsilon, int start, int end, std::vector<bool>& keep)
{   
    if (end <= start + 1) return;

    double max_dist = 0;
    int index = start;

    for (int i = start + 1; i < end; ++i) {
        // 計算第 i 點到線段 (start, end) 的笛卡兒距離
        double d = calculateDistance(points[i], points[start], points[end]);
        if (d > max_dist) {
            max_dist = d;
            index = i;
        }
    }

    if (max_dist > epsilon) {
        keep[index] = true;
        rdpRecursive(points, epsilon, start, index, keep);
        rdpRecursive(points, epsilon, index, end, keep);
    }
}

double KinematicsUtils::calculateDistance(const CartesianPose& p, const CartesianPose& a, const CartesianPose& b) 
{
    // 3D 空間中「點到線段」的垂直距離公式
    Eigen::Vector3d AB = b.position - a.position;
    Eigen::Vector3d AP = p.position - a.position;
    
    // 如果起點跟終點重合，回傳點到點距離
    if (AB.norm() < 1e-6) return AP.norm();

    // 使用外積公式計算垂直距離
    return AB.cross(AP).norm() / AB.norm();
}

CartesianPose KinematicsUtils::computeFK(const std::vector<double>& joint_positions) {
    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_positions);
    const Eigen::Isometry3d& transform = 
        kinematic_state_->getGlobalLinkTransform(base_frame_).inverse() * kinematic_state_->getGlobalLinkTransform(tip_frame_);

    CartesianPose res;
    res.position = transform.translation();
    res.orientation = Eigen::Quaterniond(transform.rotation());
    return res;
}