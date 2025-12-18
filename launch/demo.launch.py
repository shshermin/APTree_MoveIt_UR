# MoveIt launch file for real UR robot

import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    # Launch arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    end_effector_type = LaunchConfiguration("end_effector_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

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

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Note: UR robot driver should be launched separately in another terminal

    # Robot State Publisher - will get robot description from ur_robot_driver
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
            warehouse_ros_config,
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

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    # Mesh Scene Publisher - uses STL/DAE collision mesh
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

    return LaunchDescription(
        [
            # Declare arguments
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur10",
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
                    "ur18",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "robot_ip",
                description="IP address of the UR robot.",
                default_value="192.168.1.100",
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
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                description="Path where the warehouse database should be stored",
            ),
            # Launch nodes
            robot_state_publisher,
            static_tf_node,
            move_group_node,
            mesh_scene_publisher_node,
            rviz_node,
        ]
    )
