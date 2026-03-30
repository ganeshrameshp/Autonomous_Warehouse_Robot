from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share = FindPackageShare("warehouse_robot_description")

    gazebo_launch = PathJoinSubstitution(
        [package_share, "launch", "gazebo.launch.py"]
    )

    warehouse_world = PathJoinSubstitution(
        [package_share, "worlds", "warehouse.world"]
    )

    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "warehouse_slam.rviz"]
    )

    entity_name = LaunchConfiguration("entity_name")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    world = LaunchConfiguration("world")

    # Gazebo
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "entity_name": entity_name,
            "gui": gui,
            "world": world,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "z_pose": z_pose,
            "use_rviz": "false",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 🔥 RTAB-MAP (3D SLAM)
    rtabmap = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'use_sim_time': True
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points')
        ]
    )

    # Visualization
    rtabmapviz = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument("entity_name", default_value="warehouse_robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=warehouse_world),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("z_pose", default_value="0.5"),

        simulation,
        rtabmap,
        rtabmapviz,
    ])