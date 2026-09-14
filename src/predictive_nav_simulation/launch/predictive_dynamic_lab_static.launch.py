"""Static-only mapping and manual-drive world for PredictiveNav2 experiments.

This launch deliberately reuses the robot and bridge bringup while selecting
the new actor-free world.  Dynamic actor routes will be added later in a
separate trial launch so the SLAM/AMCL map remains static and reproducible.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_dir = get_package_share_directory("predictive_nav_simulation")
    base_launch = os.path.join(
        simulation_dir, "launch", "dynamic_navigation_lab.launch.py"
    )
    world_path = os.path.join(
        simulation_dir, "worlds", "predictive_dynamic_lab.sdf"
    )
    rviz_path = os.path.join(
        simulation_dir, "rviz", "predictive_dynamic_lab.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_bridge", default_value="true"),
        DeclareLaunchArgument("use_world_odom_anchor", default_value="true"),
        DeclareLaunchArgument("enable_scan_safety_guard", default_value="true"),
        # Lower-centre staging area, not inside any experimental interaction zone.
        DeclareLaunchArgument("spawn_x", default_value="-4.5"),
        DeclareLaunchArgument("spawn_y", default_value="-6.8"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "world": world_path,
                "world_name": "predictive_dynamic_lab",
                "rviz_config": rviz_path,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_bridge": LaunchConfiguration("use_bridge"),
                "use_world_odom_anchor": LaunchConfiguration("use_world_odom_anchor"),
                "enable_scan_safety_guard": LaunchConfiguration("enable_scan_safety_guard"),
                # Static-world baseline: no actor model/controller is loaded.
                "enable_dynamic_obstacle": "false",
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            }.items(),
        ),
    ])
