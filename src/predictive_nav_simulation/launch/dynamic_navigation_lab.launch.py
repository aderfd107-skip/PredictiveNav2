import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_command():
    return Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("predictive_nav_description"), "urdf", "robot.urdf.xacro"
        ]),
        " robot_name:=nav_robot robot_color:=predictive_nav_blue",
        " lidar_x:=0.22 lidar_y:=0 lidar_z:=0.20 use_gazebo_plugins:=true",
    ])


def generate_launch_description():
    sim_dir = get_package_share_directory("predictive_nav_simulation")
    world_path = os.path.join(sim_dir, "worlds", "dynamic_navigation_lab.sdf")
    rviz_config = os.path.join(sim_dir, "rviz", "dynamic_navigation_lab.rviz")
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    robot_description = robot_description_command()

    return LaunchDescription([
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        SetEnvironmentVariable("QT_OPENGL", "software"),
        DeclareLaunchArgument("world", default_value=world_path),
        DeclareLaunchArgument("gz_args", default_value=["-r ", LaunchConfiguration("world")]),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_bridge", default_value="true"),
        DeclareLaunchArgument("use_world_odom_anchor", default_value="true",
                              description="Set false when SLAM publishes map to odom."),
        DeclareLaunchArgument("enable_scan_safety_guard", default_value="true",
                              description="Temporary LiDAR emergency-stop filter for manual simulation."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim", executable="create", name="spawn_nav_robot", output="screen",
            arguments=["-world", "dynamic_navigation_lab", "-topic", "/robot_description",
                       "-name", "nav_robot", "-allow_renaming", "false", "-x", "5.8", "-y", "-3.85",
                       "-z", "0.0", "-Y", "1.5708"],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="dynamic_navigation_bridge",
            output="screen", condition=IfCondition(LaunchConfiguration("use_bridge")),
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/nav_robot/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/nav_robot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/nav_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/nav_robot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/nav_robot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/world/dynamic_navigation_lab/model/nav_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            ],
            remappings=[
                ("/nav_robot/cmd_vel_safe", "/cmd_vel_safe"),
                ("/nav_robot/odom", "/odom"),
                ("/nav_robot/tf", "/tf"),
                ("/nav_robot/scan", "/scan"),
                ("/nav_robot/imu", "/imu"),
                ("/world/dynamic_navigation_lab/model/nav_robot/joint_state", "/joint_states"),
            ],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="world_to_odom",
            output="screen", condition=IfCondition(LaunchConfiguration("use_world_odom_anchor")),
            arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0",
                       "--yaw", "0", "--frame-id", "world", "--child-frame-id", "odom"],
        ),
        Node(
            package="predictive_nav_simulation", executable="scan_safety_guard.py",
            name="scan_safety_guard", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_scan_safety_guard")),
            parameters=[{"use_sim_time": True, "stop_distance": 0.20, "slow_distance": 0.50,
                         "front_sector_degrees": 46.0, "rear_sector_degrees": 42.0,
                         "publish_rate": 30.0, "cmd_timeout": 0.0, "linear_accel_limit": 0.5,
                         "angular_accel_limit": 0.7}],
            remappings=[("cmd_vel_in", "/cmd_vel"), ("cmd_vel_out", "/cmd_vel_safe"),
                        ("scan", "/scan")],
        ),
        Node(
            package="predictive_nav_simulation", executable="static_world_marker_publisher.py",
            name="static_world_marker_publisher", output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            parameters=[{"world_file": LaunchConfiguration("world"), "use_sim_time": True}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")), arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
