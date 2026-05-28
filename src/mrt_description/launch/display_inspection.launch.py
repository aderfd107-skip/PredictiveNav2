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
                    "robot_name": "robot_2",
                    "robot_role": "inspection",
                    "robot_color": "mrt_orange",
                    "payload_enabled": "false",
                    "mast_enabled": "true",
                    "camera_enabled": "false",
                }.items(),
            )
        ]
    )
