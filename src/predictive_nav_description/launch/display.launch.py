import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("predictive_nav_description"), "urdf", "robot.urdf.xacro"
        ]),
        " robot_name:=", LaunchConfiguration("robot_name"),
        " robot_color:=", LaunchConfiguration("robot_color"),
        " use_gazebo_plugins:=false",
    ])
    rviz_config = os.path.join(
        get_package_share_directory("predictive_nav_description"), "rviz", "display.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="predictive_nav_robot"),
        DeclareLaunchArgument("robot_color", default_value="predictive_nav_blue"),
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="false",
            description="Use the optional joint_state_publisher_gui sliders when installed.",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="world_to_base_footprint", output="screen",
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0",
                       "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "world",
                       "--child-frame-id", "base_footprint"],
        ),
        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui", output="screen",
            condition=IfCondition(LaunchConfiguration("use_joint_state_publisher_gui")),
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
        ),
        # The non-GUI publisher is installed with ROS and is enough to show
        # this model's fixed geometry and zero-position wheels in RViz.
        Node(
            package="joint_state_publisher", executable="joint_state_publisher",
            name="joint_state_publisher", output="screen",
            condition=UnlessCondition(LaunchConfiguration("use_joint_state_publisher_gui")),
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
        ),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_config]),
    ])
