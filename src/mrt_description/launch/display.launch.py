import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
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
            " use_gazebo_plugins:=false",
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
            }
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
            }
        ],
    )


def static_robot_pose_node(namespace, x, y, yaw):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{namespace}_world_tf",
        output="screen",
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
            f"{namespace}/base_footprint",
        ],
    )


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("mrt_description"),
        "rviz",
        "display.rviz",
    )

    return LaunchDescription(
        [
            static_robot_pose_node(namespace="robot_1", x=-0.8, y=0.0, yaw=0.0),
            static_robot_pose_node(namespace="robot_2", x=0.8, y=0.0, yaw=0.0),
            joint_state_publisher_node(
                namespace="robot_1",
                robot_name="robot_1",
                robot_role="delivery",
                robot_color="mrt_blue",
                payload_enabled="true",
                mast_enabled="false",
                camera_enabled="true",
            ),
            robot_state_publisher_node(
                namespace="robot_1",
                robot_name="robot_1",
                robot_role="delivery",
                robot_color="mrt_blue",
                payload_enabled="true",
                mast_enabled="false",
                camera_enabled="true",
            ),
            joint_state_publisher_node(
                namespace="robot_2",
                robot_name="robot_2",
                robot_role="inspection",
                robot_color="mrt_orange",
                payload_enabled="false",
                mast_enabled="true",
                camera_enabled="false",
            ),
            robot_state_publisher_node(
                namespace="robot_2",
                robot_name="robot_2",
                robot_role="inspection",
                robot_color="mrt_orange",
                payload_enabled="false",
                mast_enabled="true",
                camera_enabled="false",
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
