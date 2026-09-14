import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def robot_description_command():
    return Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("predictive_nav_description"), "urdf", "robot.urdf.xacro"
        ]),
        " robot_name:=nav_robot robot_color:=predictive_nav_blue",
        # The xacro defaults are the shared provisional mechanical register.
        # Do not override individual dimensions here: that would decouple the
        # spawned robot from display.launch.py and future hardware calibration.
        " use_gazebo_plugins:=true",
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
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        # Kept configurable so a static mapping world can reuse this proven
        # robot / bridge / Gazebo bringup without inheriting dynamic actors.
        DeclareLaunchArgument("world_name", default_value="dynamic_navigation_lab"),
        DeclareLaunchArgument("gz_args", default_value=["-r ", LaunchConfiguration("world")]),
        DeclareLaunchArgument("spawn_x", default_value="5.8"),
        DeclareLaunchArgument("spawn_y", default_value="-3.85"),
        DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_bridge", default_value="true"),
        DeclareLaunchArgument("use_world_odom_anchor", default_value="true",
                              description="Set false when SLAM publishes map to odom."),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel_safe",
            description=(
                "ROS command topic bridged to Gazebo DiffDrive's internal "
                "nav_robot/cmd_vel_safe topic. Use /cmd_vel_safe with the "
                "manual safety guard, or /cmd_vel for direct Nav2 control."
            ),
        ),
        DeclareLaunchArgument("enable_scan_safety_guard", default_value="true",
                              description="Temporary LiDAR emergency-stop filter for manual simulation."),
        # Keep ordinary mapping and static-navigation runs reproducible.  A
        # dynamic trial opts in explicitly from nav_baseline.launch.py.
        DeclareLaunchArgument("enable_dynamic_obstacle", default_value="false",
                              description="Start the repeatable moving-obstacle controller."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{
                # Xacro emits XML, not YAML.  Preserve it verbatim for
                # robot_state_publisher on current ROS 2 launch versions.
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": True,
            }],
        ),
        Node(
            package="ros_gz_sim", executable="create", name="spawn_nav_robot", output="screen",
            arguments=["-world", LaunchConfiguration("world_name"), "-topic", "/robot_description",
                       "-name", "nav_robot", "-allow_renaming", "false", "-x", LaunchConfiguration("spawn_x"),
                       "-y", LaunchConfiguration("spawn_y"), "-z", "0.0", "-Y", LaunchConfiguration("spawn_yaw")],
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
                PythonExpression([
                    "'/world/", LaunchConfiguration("world_name"),
                    "/model/nav_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'",
                ]),
                PythonExpression([
                    "'/world/", LaunchConfiguration("world_name"),
                    "/set_pose@ros_gz_interfaces/srv/SetEntityPose'",
                ]),
            ],
            remappings=[
                # The Gazebo-side name remains cmd_vel_safe for compatibility
                # with the robot model. The ROS-side name is selectable: the
                # manual mapping launch routes it through the scan guard,
                # while Nav2 benchmarks can command DiffDrive directly.
                ("/nav_robot/cmd_vel_safe", LaunchConfiguration("cmd_vel_topic")),
                ("/nav_robot/odom", "/odom"),
                ("/nav_robot/tf", "/tf"),
                ("/nav_robot/scan", "/scan"),
                ("/nav_robot/imu", "/imu"),
                (PythonExpression([
                    "'/world/", LaunchConfiguration("world_name"), "/model/nav_robot/joint_state'",
                ]), "/joint_states"),
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
            package="predictive_nav_simulation", executable="dynamic_obstacle_controller.py",
            name="dynamic_obstacle_controller", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle")),
            parameters=[{
                "use_sim_time": True,
                "start_x": 2.25,
                "start_y": -3.70,
                "end_x": 2.25,
                "end_y": -2.00,
                "speed": 0.35,
                "update_rate": 20.0,
                "loop": True,
                "start_on_launch": True,
                "actor_name": "dynamic_obstacle_actor",
            }],
        ),
        Node(
            package="predictive_nav_simulation", executable="dynamic_obstacle_controller.py",
            name="dynamic_obstacle_center_controller", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle")),
            parameters=[{
                "use_sim_time": True,
                "actor_name": "dynamic_obstacle_center_actor",
                "start_x": -1.60,
                "start_y": 0.00,
                "end_x": 1.20,
                "end_y": 0.00,
                "speed": 0.30,
                "update_rate": 20.0,
                "loop": True,
                "start_on_launch": True,
            }],
        ),
        Node(
            package="predictive_nav_simulation", executable="dynamic_obstacle_controller.py",
            name="dynamic_obstacle_upper_controller", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle")),
            parameters=[{
                "use_sim_time": True,
                "actor_name": "dynamic_obstacle_upper_actor",
                "start_x": 2.20,
                "start_y": 2.20,
                "end_x": 5.40,
                "end_y": 2.20,
                "speed": 0.28,
                "update_rate": 20.0,
                "loop": True,
                "start_on_launch": True,
            }],
        ),
        Node(
            package="predictive_nav_simulation", executable="static_world_marker_publisher.py",
            name="static_world_marker_publisher", output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            parameters=[{"world_file": LaunchConfiguration("world"), "use_sim_time": True}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
