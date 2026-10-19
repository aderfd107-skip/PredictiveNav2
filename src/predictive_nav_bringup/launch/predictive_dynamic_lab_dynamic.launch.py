"""Dynamic-actor trial on the new predictive_dynamic_lab map.

The static navigation launch remains unchanged. This wrapper reuses it and
starts deterministic Gazebo actor controllers only for the dynamic trial.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("predictive_nav_bringup")
    static_launch = os.path.join(
        bringup_dir, "launch", "predictive_dynamic_lab_nav.launch.py"
    )
    world_name = "predictive_dynamic_lab"

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_launch)),
        Node(
            package="predictive_nav_simulation",
            executable="dynamic_obstacle_controller.py",
            name="dynamic_obstacle_controller",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "world_name": world_name,
                "actor_name": "dynamic_obstacle_actor",
                "start_x": -6.80, "start_y": -1.00,
                "end_x": -3.50, "end_y": -1.00,
                "speed": 0.35, "update_rate": 20.0,
                "loop": True, "start_on_launch": True,
            }],
        ),
        Node(
            package="predictive_nav_simulation",
            executable="dynamic_obstacle_controller.py",
            name="dynamic_obstacle_center_controller",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "world_name": world_name,
                "actor_name": "dynamic_obstacle_center_actor",
                "start_x": -1.60, "start_y": 0.00,
                "end_x": 1.20, "end_y": 0.00,
                "speed": 0.30, "update_rate": 20.0,
                "loop": True, "start_on_launch": True,
            }],
        ),
        Node(
            package="predictive_nav_simulation",
            executable="dynamic_obstacle_controller.py",
            name="dynamic_obstacle_upper_controller",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "world_name": world_name,
                "actor_name": "dynamic_obstacle_upper_actor",
                "start_x": 2.20, "start_y": 2.20,
                "end_x": 5.40, "end_y": 2.20,
                "speed": 0.28, "update_rate": 20.0,
                "loop": True, "start_on_launch": True,
            }],
        ),
    ])
