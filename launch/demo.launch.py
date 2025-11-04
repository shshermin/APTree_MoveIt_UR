# Standalone demo launch file for MoveIt with RViz
# No real robot or Gazebo needed - uses fake controllers for visualization

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    ur_type = LaunchConfiguration("ur_type")
    end_effector_type = LaunchConfiguration("end_effector_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="hello_moveit")
        .robot_description(
            file_path=Path("urdf") / "ur.urdf.xacro",
            mappings={
                "name": ur_type,
                "ur_type": ur_type,
                "end_effector_type": end_effector_type,
            }
        )
        .robot_description_semantic(
            file_path=Path("config") / "srdf" / "ur.srdf.xacro",
            mappings={
                "name": ur_type,
                "end_effector_type": end_effector_type,
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp", "chomp"]
        )
        .to_moveit_configs()
    )

    # Robot State Publisher - publishes robot description and TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hello_moveit"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF for world frame
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher GUI - allows manual joint control
    # Note: It will start at zeros, but you can use the GUI sliders to move joints
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    # Scene Publisher (Box primitive) - fallback/example
    scene_publisher_node = Node(
        package="hello_moveit",
        executable="publish_scene.py",
        name="scene_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Mesh Scene Publisher - uses STL/DAE collision mesh
    # Replace the path below with your actual mesh file path
    mesh_scene_publisher_node = Node(
        package="hello_moveit",
        executable="publish_scene_mesh.py",
        name="mesh_scene_publisher",
        output="screen",
        parameters=[
            {"frame_id": "base_link"},
            {"mesh_path": "/home/shermin/ws_moveit/src/hello_moveit/meshes/collision/robottable.stl"},
            {"scale_x": 1.0}, {"scale_y": 1.0}, {"scale_z": 1.0},
            {"pos_x": 0.0}, {"pos_y": 0.0}, {"pos_z": -0.001},
            {"quat_x": 0.0}, {"quat_y": 0.0}, {"quat_z": 0.0}, {"quat_w": 1.0},
        ],
    )

    # Planning Scene Publisher - loads static scene objects (table, etc.)
    planning_scene_file = PathJoinSubstitution(
        [FindPackageShare("hello_moveit"), "config", "planning_scene.yaml"]
    )
    
    # Node to publish planning scene from file
    planning_scene_publisher = Node(
        package="moveit_ros_planning",
        executable="moveit_publish_scene_from_text",
        name="planning_scene_publisher",
        output="screen",
        arguments=[planning_scene_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            # Declare arguments
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur10e",
                description="Type/series of used UR robot.",
                choices=[
                    "ur3",
                    "ur5",
                    "ur10",
                    "ur3e",
                    "ur5e",
                    "ur7e",
                    "ur10e",
                    "ur12e",
                    "ur16e",
                    "ur8long",
                    "ur15",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "end_effector_type",
                default_value="none",
                description="Type of end-effector attached to the robot.",
                choices=["none", "gripper", "nailgun"],
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz?",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            # Launch nodes
            robot_state_publisher,
            static_tf_node,
            joint_state_publisher_gui,
            move_group_node,
            mesh_scene_publisher_node,
            rviz_node,
        ]
    )
