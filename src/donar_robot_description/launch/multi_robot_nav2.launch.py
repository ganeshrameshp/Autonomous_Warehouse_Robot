import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    slam = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    bringup_launch = os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")

    robot1_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        condition=UnlessCondition(slam),
        launch_arguments={
            "namespace": "robot1",
            "use_namespace": "True",
            "slam": "False",
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    robot2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        condition=UnlessCondition(slam),
        launch_arguments={
            "namespace": "robot2",
            "use_namespace": "True",
            "slam": "False",
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    robot1_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        condition=IfCondition(slam),
        launch_arguments={
            "namespace": "robot1",
            "use_namespace": "True",
            "slam": "True",
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    robot2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        condition=IfCondition(slam),
        launch_arguments={
            "namespace": "robot2",
            "use_namespace": "True",
            "slam": "True",
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition the Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Use slam_toolbox through Nav2 bringup instead of map+AMCL.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="YAML map file used when slam:=false.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    pkg_share, "config", "nav2_params_multi_robot.yaml"
                ),
                description="Shared Nav2 parameter file with robot1/robot2 namespaces.",
            ),
            robot1_localization,
            robot2_localization,
            robot1_slam,
            robot2_slam,
        ]
    )
