"""Known-map, static Nav2 baseline for the Predictive Dynamic Lab.

This is the first navigation stage after saving ``predictive_dynamic_lab``.
It deliberately contains no moving actor: DWB / AMCL performance measured
here is the static reference for later original-MPPI and Predictive-MPPI
experiments. The saved map and Gazebo world are selected as one pair.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("predictive_nav_bringup")
    simulation_dir = get_package_share_directory("predictive_nav_simulation")

    baseline_launch = os.path.join(bringup_dir, "launch", "nav_baseline.launch.py")
    truth_map_path = os.path.join(
        simulation_dir, "maps", "predictive_dynamic_lab_truth.yaml"
    )
    world_path = os.path.join(simulation_dir, "worlds", "predictive_dynamic_lab.sdf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start the Nav2 RViz configuration.",
        ),
        DeclareLaunchArgument(
            "map", default_value=truth_map_path,
            description=(
                "Static map for AMCL/Nav2. Defaults to the SDF-derived truth "
                "map; the original saved SLAM map remains selectable."
            ),
        ),
        # Keep the normal staging pose as the default, while allowing each
        # static regression route to start from a reproducible map position.
        DeclareLaunchArgument("spawn_x", default_value="-4.5"),
        DeclareLaunchArgument("spawn_y", default_value="-6.8"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(baseline_launch),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "world": world_path,
                "world_name": "predictive_dynamic_lab",
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                # Nav2's obstacle layers already consume /scan.  Do not add
                # the manual teleop guard here: it can zero angular velocity
                # when a wall enters the forward scan sector at a corner.
                "enable_scan_safety_guard": "false",
                "cmd_vel_topic": "/cmd_vel",
                # Use the canonical DWB baseline from nav2_dwb.yaml. Any
                # corner-specific tuning must first be verified in isolation,
                # rather than changing navigation behavior across this map.
                "enable_dynamic_obstacle": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
    ])
