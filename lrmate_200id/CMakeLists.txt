cmake_minimum_required(VERSION 3.16)
project(lrmate_200id)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

set(DEPS
  rclcpp
  trajectory_msgs
  kdl_parser
  orocos_kinematics_dynamics
)


install(
  DIRECTORY bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}
)


# Install URDF and description files
install(
  DIRECTORY description/launch description/urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
