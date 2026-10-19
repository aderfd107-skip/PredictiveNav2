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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            "enable_corner_alignment_goal_bridge", default_value="true",
            description="Accept RViz corner-aware goals on /corner_alignment/goal.",
        ),
        DeclareLaunchArgument(
            "map", default_value=truth_map_path,
            description=(
                "Static map for AMCL/Nav2. Defaults to the SDF-derived truth "
                "map; the original saved SLAM map remains selectable."
            ),
        ),
        DeclareLaunchArgument(
            "params_file", default_value=dwb_params_path,
            description=(
                "Nav2 parameter YAML. Defaults to the DWB static baseline; "
                "the dedicated MPPI launch overrides this with nav2_mppi_baseline.yaml."
            ),
        ),
        # Keep the normal staging pose as the default, while allowing each
        # static regression route to start from a reproducible map position.
        DeclareLaunchArgument("spawn_x", default_value="-4.5"),
        DeclareLaunchArgument("spawn_y", default_value="-6.8"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "enable_scan_safety_guard", default_value="true",
            description="Enable the final LiDAR speed guard before Gazebo.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel_safe",
            description="Velocity topic bridged to the Gazebo differential drive.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(baseline_launch),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "map": LaunchConfiguration("map"),
                "world": world_path,
                "world_name": "predictive_dynamic_lab",
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                # Nav2 plans with /scan and the final scan guard protects
                # Gazebo/real DiffDrive from physical contact.  The guard
                # blocks unsafe translation but leaves pure rotation enabled
                # for the circular base at a corner.
                "enable_scan_safety_guard": "true",
                "cmd_vel_topic": "/cmd_vel_safe",
                # Use the canonical DWB baseline from nav2_dwb.yaml. Any
                # corner-specific tuning must first be verified in isolation,
                # rather than changing navigation behavior across this map.
                "enable_dynamic_obstacle": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
        Node(
            package="predictive_nav_bringup",
            executable="corner_alignment_goal_bridge.py",
            name="corner_alignment_goal_bridge",
            output="screen",
            condition=IfCondition(
                LaunchConfiguration("enable_corner_alignment_goal_bridge")
            ),
            parameters=[{
                "goal_topic": "/corner_alignment/goal",
                "goal_frame": "map",
            }],
        ),
    ])
