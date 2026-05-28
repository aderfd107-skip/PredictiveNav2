import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_command(
    robot_name,
    robot_role,
    robot_color,
    payload_enabled,
    mast_enabled,
    camera_enabled,
):
    return Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare("mrt_description"), "urdf", "robot.urdf.xacro"]
            ),
            " robot_name:=",
            robot_name,
            " robot_role:=",
            robot_role,
            " robot_color:=",
            robot_color,
            " payload_enabled:=",
            payload_enabled,
            " mast_enabled:=",
            mast_enabled,
            " camera_enabled:=",
            camera_enabled,
            " use_gazebo_plugins:=true",
        ]
    )


def robot_state_publisher_node(
    namespace,
    robot_name,
    robot_role,
    robot_color,
    payload_enabled,
    mast_enabled,
    camera_enabled,
):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_robots")),
        parameters=[
            {
                "frame_prefix": f"{namespace}/",
                "robot_description": robot_description_command(
                    robot_name,
                    robot_role,
                    robot_color,
                    payload_enabled,
                    mast_enabled,
                    camera_enabled,
                ),
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("joint_states", f"/{namespace}/joint_states"),
            ("robot_description", f"/{namespace}/robot_description"),
        ],
    )


def joint_state_publisher_node(
    namespace,
    robot_name,
    robot_role,
    robot_color,
    payload_enabled,
    mast_enabled,
    camera_enabled,
):
    return Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=namespace,
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_robots")),
        parameters=[
            {
                "robot_description": robot_description_command(
                    robot_name,
                    robot_role,
                    robot_color,
                    payload_enabled,
                    mast_enabled,
                    camera_enabled,
                ),
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("joint_states", f"/{namespace}/joint_states"),
            ("robot_description", f"/{namespace}/robot_description"),
        ],
    )


def spawn_robot_node(namespace, x, y, yaw):
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{namespace}",
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_robots")),
        arguments=[
            "-world",
            "office_service_mvp",
            "-topic",
            f"/{namespace}/robot_description",
            "-name",
            namespace,
            "-allow_renaming",
            "false",
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            "0.0",
            "-Y",
            str(yaw),
        ],
    )


def static_odom_anchor_node(namespace, x, y, yaw):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{namespace}_odom_anchor",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("spawn_robots"),
                LaunchConfiguration("use_world_odom_anchors"),
            )
        ),
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            str(yaw),
            "--frame-id",
            "world",
            "--child-frame-id",
            f"{namespace}/odom",
        ],
    )


def ros_gz_bridge_node():
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="office_service_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_bridge")),
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/robot_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/robot_1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/robot_1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/robot_1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/robot_1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/robot_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/robot_2/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/robot_2/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/robot_2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/robot_2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ("/robot_1/tf", "/tf"),
            ("/robot_2/tf", "/tf"),
        ],
    )


def rviz_node(rviz_config):
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )


def office_map_marker_node(world_path):
    return Node(
        package="mrt_simulation",
        executable="office_map_marker_publisher.py",
        name="office_map_marker_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            {
                "world_file": world_path,
                "use_sim_time": True,
            }
        ],
    )


def generate_launch_description():
    sim_dir = get_package_share_directory("mrt_simulation")
    world_path = os.path.join(sim_dir, "worlds", "office_service_mvp.sdf")
    rviz_config = os.path.join(sim_dir, "rviz", "office_service_mvp.rviz")
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    return LaunchDescription(
        [
            # Gazebo GUI can flicker or go blank on some VM / Wayland / GPU setups.
            # Software OpenGL is slower but much more stable for this lightweight map.
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable("QT_OPENGL", "software"),
            DeclareLaunchArgument(
                "world",
                default_value=world_path,
                description="Absolute path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value=["-r ", LaunchConfiguration("world")],
                description="Arguments passed to Gazebo Sim.",
            ),
            DeclareLaunchArgument(
                "spawn_robots",
                default_value="true",
                description="Spawn the two service robots into the Gazebo world.",
            ),
            DeclareLaunchArgument(
                "use_bridge",
                default_value="true",
                description="Bridge Gazebo topics to ROS 2 topics for control and visualization.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the office service visualization config.",
            ),
            DeclareLaunchArgument(
                "use_world_odom_anchors",
                default_value="true",
                description=(
                    "Publish world->robot odom static anchors for plain RViz visualization. "
                    "Set false when slam_toolbox publishes map->odom."
                ),
            ),
            DeclareLaunchArgument(
                "robot_1_payload_enabled",
                default_value="true",
                description="Enable robot_1 payload geometry.",
            ),
            DeclareLaunchArgument(
                "robot_2_payload_enabled",
                default_value="false",
                description="Enable robot_2 payload geometry.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
            ),
            ros_gz_bridge_node(),
            office_map_marker_node(LaunchConfiguration("world")),
            static_odom_anchor_node(namespace="robot_1", x=5.8, y=-3.85, yaw=1.5708),
            static_odom_anchor_node(namespace="robot_2", x=4.5, y=-3.85, yaw=1.5708),
            joint_state_publisher_node(
                namespace="robot_1",
                robot_name="robot_1",
                robot_role="delivery",
                robot_color="mrt_blue",
                payload_enabled=LaunchConfiguration("robot_1_payload_enabled"),
                mast_enabled="false",
                camera_enabled="true",
            ),
            robot_state_publisher_node(
                namespace="robot_1",
                robot_name="robot_1",
                robot_role="delivery",
                robot_color="mrt_blue",
                payload_enabled=LaunchConfiguration("robot_1_payload_enabled"),
                mast_enabled="false",
                camera_enabled="true",
            ),
            spawn_robot_node(namespace="robot_1", x=5.8, y=-3.85, yaw=1.5708),
            joint_state_publisher_node(
                namespace="robot_2",
                robot_name="robot_2",
                robot_role="inspection",
                robot_color="mrt_orange",
                payload_enabled=LaunchConfiguration("robot_2_payload_enabled"),
                mast_enabled="true",
                camera_enabled="false",
            ),
            robot_state_publisher_node(
                namespace="robot_2",
                robot_name="robot_2",
                robot_role="inspection",
                robot_color="mrt_orange",
                payload_enabled=LaunchConfiguration("robot_2_payload_enabled"),
                mast_enabled="true",
                camera_enabled="false",
            ),
            spawn_robot_node(namespace="robot_2", x=4.5, y=-3.85, yaw=1.5708),
            rviz_node(rviz_config),
        ]
    )
