import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="True",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    moveit_config = (
        MoveItConfigsBuilder("lrmate_200id", package_name="lrmate_200id")
        .robot_description(file_path="urdf/lrmate_200id.urdf.xacro")
        .robot_description_semantic(file_path="config/lrmate_200id.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")   
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        remappings=[
            ("/robot_description", "robot_description"),
        ]
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 3. 啟動你的 Fanuc Node (作為 FollowJointTrajectory 的 Server)
    fanuc_bridge_node = Node(
        package="bt_fan_conn",
        executable="fanuc_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # 4. 啟動行為樹執行節點 (BT Executor)
    # bt_executor_node = Node(
    #     package="bt_fan_conn",
    #     executable="follow_path_client",
    #     output="screen",
    #     # 行為樹節點也需要讀取 robot_description 才能與 MoveIt 通訊
    #     parameters=[moveit_config.to_dict(),
    #                 {"use_sim_time": False}
    #     ] 
    # )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lrmate_200id_model_description"), "lrmate_200id/rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # lrmate_200id_world_frame = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=[
    #         '--x', '0', '--y', '0', '--z', '0.169',
    #         '--yaw', '0', '--pitch', '0', '--roll',
    #         '0', '--frame-id', 'base_link', '--child-frame-id', 'lrmate_200id_world'
    #     ]
    # )

    nodes = [
        node_robot_state_publisher,
        run_move_group_node,
        fanuc_bridge_node,
        # bt_executor_node,
        rviz_node,
        # lrmate_200id_world_frame,
    ]

    return LaunchDescription(
        declared_arguments + nodes
    )