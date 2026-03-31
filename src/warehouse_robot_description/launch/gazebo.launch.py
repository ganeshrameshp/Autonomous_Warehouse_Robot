import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share_dir = get_package_share_directory("warehouse_robot_description")
    package_model_dir = os.path.dirname(package_share_dir)
    workspace_install_dir = os.path.dirname(
        os.path.dirname(os.path.dirname(package_share_dir))
    )
    package_share = FindPackageShare("warehouse_robot_description")
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )
    urdf_file = PathJoinSubstitution(
        [package_share, "urdf", "warehouse_robot.urdf"]
    )
    pointcloud_to_scan_config = PathJoinSubstitution(
        [package_share, "config", "pointcloud_to_laserscan.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "warehouse_robot.rviz"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("warehouse_robot_description"), "worlds", "warehouse.world"]
    )

    robot_description = {
        "robot_description": ParameterValue(Command(["cat ", urdf_file]), value_type=str)
    }
    entity_name = LaunchConfiguration("entity_name")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    use_rviz = LaunchConfiguration("use_rviz")
    use_scan_conversion = LaunchConfiguration("use_scan_conversion")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time_param}],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[pointcloud_to_scan_config, {"use_sim_time": use_sim_time_param}],
        remappings=[
            ("cloud_in", "/velodyne_points"),
            ("scan", "/scan"),
        ],
        condition=IfCondition(use_scan_conversion),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_rviz),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_warehouse_robot",
        output="screen",
        arguments=[
            "-entity",
            entity_name,
            "-file",
            urdf_file,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
    )

    teleop = ExecuteProcess(
        cmd=[
            "x-terminal-emulator",
            "-T",
            "Warehouse Robot Teleop",
            "-e",
            "bash",
            "-lc",
            (
                "source /opt/ros/humble/setup.bash && "
                f"source {workspace_install_dir}/setup.bash && "
                "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
            ),
        ],
        condition=IfCondition(use_teleop),
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "gui": gui,
            "world": world,
        }.items(),
    )

    gazebo_model_path = os.pathsep.join(
        [
            package_model_dir,
            "/usr/share/gazebo-11/models",
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ]
    )
    gazebo_resource_path = os.pathsep.join(
        [
            package_share_dir,
            "/usr/share/gazebo-11",
            "/usr/share/gazebo-11/worlds",
            os.environ.get("GAZEBO_RESOURCE_PATH", ""),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "entity_name",
            default_value="warehouse_robot",
            description="Entity name used when spawning the robot in Gazebo.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo client GUI.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Absolute path to the Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "x_pose",
            default_value="0.0",
            description="Initial x position of the robot in Gazebo.",
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value="0.0",
            description="Initial y position of the robot in Gazebo.",
        ),
        DeclareLaunchArgument(
            "z_pose",
            default_value="0.2",
            description="Initial z position of the robot in Gazebo.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with robot, TF, point cloud, and scan displays.",
        ),
        DeclareLaunchArgument(
            "use_scan_conversion",
            default_value="true",
            description="Convert the simulated PointCloud2 into a 2D LaserScan.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time.",
        ),
        DeclareLaunchArgument(
            "use_teleop",
            default_value="true",
            description="Open a terminal and start keyboard teleop automatically.",
        ),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", gazebo_resource_path),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        pointcloud_to_laserscan,
        rviz,
        teleop,
    ])