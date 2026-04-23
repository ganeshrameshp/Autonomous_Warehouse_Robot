import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")
    slam_params = os.path.join(pkg_share, "config", "slam_params_3d_lidar.yaml")

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_name = LaunchConfiguration("robot_name")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "gazebo_sdf.launch.py")
        ),
        launch_arguments={
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "robot_name": robot_name,
        }.items(),
    )

    slam_launch = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": True,
                "scan_topic": ParameterValue(["/", robot_name, "/scan"], value_type=str),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "launch", "slam.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="-10.9"),
            DeclareLaunchArgument("spawn_y", default_value="-5.45"),
            DeclareLaunchArgument("spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("robot_name", default_value="donar_robot"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            sim_launch,
            TimerAction(
                period=8.0,
                actions=[
                    slam_launch,
                    rviz,
                ],
            ),
        ]
    )
