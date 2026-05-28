from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mrt_description"), "launch", "display_single.launch.py"]
                    )
                ),
                launch_arguments={
                    "robot_name": "robot_1",
                    "robot_role": "delivery",
                    "robot_color": "mrt_blue",
                    "payload_enabled": "true",
                    "mast_enabled": "false",
                    "camera_enabled": "true",
                }.items(),
            )
        ]
    )
