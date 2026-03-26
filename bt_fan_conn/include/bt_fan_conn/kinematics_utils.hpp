#include <vector>
#include <Eigen/Geometry>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class KinematicsUtils {
public:
    using TrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    
    /**
     * @brief 儲存笛卡兒空間的位姿資料 (XYZ + Quaternion)
     */
    struct CartesianPose {
        Eigen::Vector3d position; ///< 單位：m
        Eigen::Quaterniond orientation; ///< Quaternion
    };

    /**
    * @brief 建構函式，增加參數化配置
     * @param node ROS2 節點
     * @param group_name MoveIt Planning Group 名稱 (如 "lrmate_200id")
     * @param base_frame 基座坐標系名稱 (如 "base_link" 或 "world")
     * @param tip_frame 末端工具坐標系名稱 (如 "tool0")
     */
    KinematicsUtils(const rclcpp::Node::SharedPtr& node,
        const std::string& group_name,
        const std::string& base_frame,
        const std::string& tip_frame);

    /**
     * @brief 將關節空間軌跡轉換並簡化為笛卡兒空間軌跡 (RDP 演算法)
     * @details 此函式會先對所有點進行正向運動學(FK)，隨後根據 epsilon 門檻值過濾冗餘點。
     * @param points 原始關節空間軌跡點 (joint positions)
     * @param epsilon 容許的最大線性偏差 (單位：mm)。若點到線段距離小於此值則捨棄。
     * @return std::vector<CartesianPose> 簡化後的笛卡兒位姿序列
     */
    std::vector<CartesianPose> simplifyTrajectory(
        const std::vector<TrajectoryPoint>& points, 
        double epsilon);

    /**
     * @brief RDP 演算法的遞迴核心實作 (靜態函式)
     * @param poses 已轉換為 CartesianPose 的完整點序列
     * @param epsilon 距離門檻值
     * @param start 當前檢查區段的起始索引
     * @param end 當前檢查區段的結束索引
     * @param keep 布林陣列，記錄哪些索引處的點應被保留
     */
    static void rdpRecursive(const std::vector<CartesianPose>& points, 
        double epsilon, int start, int end, std::vector<bool>& keep);

    /**
     * @brief 計算 3D 空間中點到線段的垂直距離 (靜態函式)
     * @details 使用向量公式： \f$ d = \frac{\|(\vec{P}-\vec{A}) \times (\vec{P}-\vec{B})\|}{\|\vec{B}-\vec{A}\|} \f$
     * @param p 目標點 P
     * @param start 線段起點 A
     * @param end 線段終點 B
     * @return double 垂直距離 (mm)
     */
    static double calculateDistance(const CartesianPose& p, const CartesianPose& start, const CartesianPose& end);

    /**
     * @brief 執行正向運動學 (FK)，計算特定關節角度對應的 TCP 位姿
     * @param joint_positions 各關節角度 (單位：rad)
     * @return CartesianPose 轉換後的笛卡兒座標與四元數 (m, quaternion)
     */
    CartesianPose computeFK(const std::vector<double>& joint_positions);

private:
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    std::string planning_group_;
    std::string base_frame_;
    std::string tip_frame_;
    const moveit::core::JointModelGroup* joint_model_group_;
    
    static constexpr double RAD2DEG = 180.0 / M_PI;
};