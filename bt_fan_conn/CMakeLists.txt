cmake_minimum_required(VERSION 3.16)
project(bt_fan_conn)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)

set(DEPS
  behaviortree_ros2
  std_msgs
  std_srvs
  btcpp_ros2_interfaces
  robot_interfaces
)

# Build an Action Client that calls the sleep action (STATIC version)
add_executable(parallel_gripper_client
  src/parallel_gripper_action.cpp
  src/parallel_gripper_client.cpp)

######################################################
# Build a client that call the sleep action (Plugin version)
add_library(parallel_gripper_plugin SHARED src/parallel_gripper_action.cpp)
target_compile_definitions(parallel_gripper_plugin PRIVATE BT_PLUGIN_EXPORT)

add_executable(parallel_gripper_client_dyn src/parallel_gripper_client.cpp)
target_compile_definitions(parallel_gripper_client_dyn PRIVATE USE_PARALLEL_GRIPPER_PLUGIN)

add_executable(parallel_gripper_server src/parallel_gripper_server.cpp)

######################################################
# set include directory
set(INC_DIR "${PROJECT_SOURCE_DIR}/include")

# 使用 PUBLIC，確保 link 這個 library 的人也能自動拿到 include 路徑
target_include_directories(parallel_gripper_plugin PUBLIC 
  $<BUILD_INTERFACE:${INC_DIR}>
  $<INSTALL_INTERFACE:include>
)

# 為其他執行檔設定路徑
target_include_directories(parallel_gripper_client PRIVATE ${INC_DIR})
target_include_directories(parallel_gripper_client_dyn PRIVATE ${INC_DIR})
target_include_directories(parallel_gripper_server PRIVATE ${INC_DIR})

######################################################
# ament_target_dependencies

foreach(target 
    parallel_gripper_plugin 
    parallel_gripper_client 
    parallel_gripper_client_dyn 
    parallel_gripper_server)
    
  ament_target_dependencies(${target} ${DEPS})
endforeach()

target_link_libraries(parallel_gripper_client_dyn parallel_gripper_plugin)

######################################################
# INSTALL

install(TARGETS
  parallel_gripper_client
  parallel_gripper_client_dyn
  parallel_gripper_server
  parallel_gripper_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS parallel_gripper_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(${DEPS})
ament_package()