import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
            "launch_sim": "false",
            "nav2_start_delay": "0.0",
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            "publish_initial_pose": "true",
        }.items(),
    )
    
    mission_planner = Node(
        package="donar_robot_description",
        executable="mission_planner.py",
        name="predefined_multi_goal_mission_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "goals_file": LaunchConfiguration("goals_file"),
                "start_delay_sec": LaunchConfiguration("mission_start_delay"),
                "max_retries": LaunchConfiguration("max_retries"),
                "stop_on_failure": False,
                "wait_for_initial_pose": True,
                "initial_pose_topic": "/initialpose",
                "amcl_pose_topic": "/amcl_pose",
                "goal_markers_topic": "/mission_goals",
                "status_topic": "/mission_status",
            }
        ],
    )
    
    delayed_nav2_launch = TimerAction(
        period=15.0,
        actions=[nav2_launch],
    )
    
    delayed_mission_planner = TimerAction(
        period=26.0,
        actions=[mission_planner],
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
            DeclareLaunchArgument("spawn_x", default_value="-1.0"),
            DeclareLaunchArgument("spawn_y", default_value="-3.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_x", default_value="-1.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="-3.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("mission_start_delay", default_value="10.0"),
            DeclareLaunchArgument("max_retries", default_value="2"),
            gazebo_launch,
            delayed_nav2_launch,
            delayed_mission_planner,
        ]
    )
 
