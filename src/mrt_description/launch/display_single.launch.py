import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("mrt_description"),
        "rviz",
        "display.rviz",
    )

    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare("mrt_description"), "urdf", "robot.urdf.xacro"]
            ),
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " robot_role:=",
            LaunchConfiguration("robot_role"),
            " robot_color:=",
            LaunchConfiguration("robot_color"),
            " payload_enabled:=",
            LaunchConfiguration("payload_enabled"),
            " mast_enabled:=",
            LaunchConfiguration("mast_enabled"),
            " camera_enabled:=",
            LaunchConfiguration("camera_enabled"),
            " use_gazebo_plugins:=false",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot_1"),
            DeclareLaunchArgument("robot_role", default_value="delivery"),
            DeclareLaunchArgument("robot_color", default_value="mrt_blue"),
            DeclareLaunchArgument("payload_enabled", default_value="true"),
            DeclareLaunchArgument("mast_enabled", default_value="false"),
            DeclareLaunchArgument("camera_enabled", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_footprint",
                output="screen",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "base_footprint",
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
