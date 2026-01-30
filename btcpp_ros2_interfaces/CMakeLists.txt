cmake_minimum_required(VERSION 3.16)
project(btcpp_ros2_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(btcpp_ros2_interfaces
    "msg/NodeStatus.msg"
    "srv/GetTrees.srv"
    "action/ExecuteTree.action"
    "action/MoveTo.action"
    "action/ParallelGripper.action"
    "action/Sleep.action"
    DEPENDENCIES geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
