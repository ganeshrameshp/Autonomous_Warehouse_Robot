import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "gazebo_sdf.launch.py")
        ),
        launch_arguments={
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "robot_name": "donar_robot",
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "single_robot_nav2.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
        }.items(),
    )

    mission_planner = Node(
        package="donar_robot_description",
        executable="mission_planner.py",
        name="final_demo_mission_planner",
        output="screen",
        parameters=[
            {
                "goals_file": LaunchConfiguration("goals_file"),
                "start_delay_sec": LaunchConfiguration("mission_start_delay"),
                "max_retries": LaunchConfiguration("max_retries"),
                "stop_on_failure": False,
                "status_topic": "/mission_status",
            }
        ],
    )

    dynamic_obstacle = Node(
        package="donar_robot_description",
        executable="dynamic_obstacle_controller.py",
        name="final_demo_dynamic_obstacle",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle")),
        parameters=[
            {
                "world_name": "warehouse_world",
                "start_delay_sec": LaunchConfiguration("obstacle_start_delay"),
                "start_x": LaunchConfiguration("obstacle_start_x"),
                "start_y": LaunchConfiguration("obstacle_start_y"),
                "end_x": LaunchConfiguration("obstacle_end_x"),
                "end_y": LaunchConfiguration("obstacle_end_y"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="/home/ganesh/robot_ws/warehouse_map.yaml",
            ),
            DeclareLaunchArgument(
                "goals_file",
                default_value=os.path.join(
                    pkg_share, "config", "final_demo_goals.json"
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="-9.6"),
            DeclareLaunchArgument("spawn_y", default_value="-4.9"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("initial_pose_x", default_value="-9.6"),
            DeclareLaunchArgument("initial_pose_y", default_value="-4.9"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="1.57"),
            DeclareLaunchArgument("mission_start_delay", default_value="18.0"),
            DeclareLaunchArgument("max_retries", default_value="2"),
            DeclareLaunchArgument("enable_dynamic_obstacle", default_value="true"),
            DeclareLaunchArgument("obstacle_start_delay", default_value="30.0"),
            DeclareLaunchArgument("obstacle_start_x", default_value="-1.5"),
            DeclareLaunchArgument("obstacle_start_y", default_value="-0.5"),
            DeclareLaunchArgument("obstacle_end_x", default_value="2.5"),
            DeclareLaunchArgument("obstacle_end_y", default_value="-0.5"),
            gazebo_launch,
            nav2_launch,
            mission_planner,
            dynamic_obstacle,
        ]
    )
