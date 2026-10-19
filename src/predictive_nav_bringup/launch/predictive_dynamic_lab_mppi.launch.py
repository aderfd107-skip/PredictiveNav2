"""Original-MPPI baseline on the corrected Predictive Dynamic Lab map.

This launch keeps the corrected SDF world and truth map together, and uses
Nav2's native MPPI controller without the project's future DynamicRiskCritic.
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
    truth_map = os.path.join(
        simulation_dir, "maps", "predictive_dynamic_lab_truth.yaml"
    )
    world = os.path.join(simulation_dir, "worlds", "predictive_dynamic_lab.sdf")
    default_params = os.path.join(
        bringup_dir, "config", "nav2_mppi_baseline.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("spawn_x", default_value="-4.5"),
        DeclareLaunchArgument("spawn_y", default_value="-6.8"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(baseline_launch),
            launch_arguments={
                "map": truth_map,
                "world": world,
                "world_name": "predictive_dynamic_lab",
                "params_file": LaunchConfiguration("params_file"),
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                "enable_scan_safety_guard": "false",
                "cmd_vel_topic": "/cmd_vel",
                "enable_dynamic_obstacle": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
        Node(
            package="predictive_nav_bringup",
            executable="corner_alignment_goal_bridge.py",
            name="corner_alignment_goal_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            parameters=[{
                "goal_topic": "/corner_alignment/goal",
                "goal_frame": "map",
            }],
        ),
    ])
