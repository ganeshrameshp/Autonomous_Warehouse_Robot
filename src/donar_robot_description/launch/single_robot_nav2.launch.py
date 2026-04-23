import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    fastdds_profile = os.path.join(pkg_share, "config", "fastdds_no_shm.xml")

    bringup_launch = os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="FASTRTPS_DEFAULT_PROFILES_FILE",
                value=fastdds_profile,
            ),
            Node(
                package="donar_robot_description",
                executable="cmd_vel_relay.py",
                name="single_robot_cmd_vel_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/donar_robot/cmd_vel",
                    }
                ],
            ),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ganesh/robot_ws/warehouse_map.yaml",
                description="Absolute path to the saved map YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "initial_pose_x",
                default_value="-11.4",
                description="Initial pose X in the map frame.",
            ),
            DeclareLaunchArgument(
                "initial_pose_y",
                default_value="-4.9",
                description="Initial pose Y in the map frame.",
            ),
            DeclareLaunchArgument(
                "initial_pose_yaw",
                default_value="-1.57",
                description="Initial pose yaw in radians.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    pkg_share, "config", "nav2_params_single_robot.yaml"
                ),
                description="Nav2 parameters for the single donar_robot setup.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the single robot navigation config.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    "slam": "False",
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                    "use_composition": "False",
                    "use_respawn": "False",
                    "log_level": "info",
                }.items(),
            ),
            Node(
                package="donar_robot_description",
                executable="nav2_activator.py",
                name="single_robot_nav2_activator",
                output="screen",
                parameters=[
                    {
                        "startup_delay_sec": 20.0,
                        "service_timeout_sec": 12.0,
                        "max_retries": 8,
                        "wait_for_all_services_sec": 40.0,
                    }
                ],
            ),
            Node(
                package="donar_robot_description",
                executable="initial_pose_publisher.py",
                name="single_robot_initial_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "topic": "/initialpose",
                        "frame_id": "map",
                        "x": LaunchConfiguration("initial_pose_x"),
                        "y": LaunchConfiguration("initial_pose_y"),
                        "yaw": LaunchConfiguration("initial_pose_yaw"),
                        "publish_count": 15,
                        "startup_delay_sec": 8.0,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(pkg_share, "launch", "single_robot_nav2.rviz")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
