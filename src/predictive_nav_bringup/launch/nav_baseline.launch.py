"""Known-map AMCL + Nav2 DWB baseline for PredictiveNav2.

This is deliberately the non-predictive control condition.  It starts the
existing Gazebo Harmonic simulation, localizes with AMCL against the saved
static map, and runs Nav2 with DWB.  Future MPPI and DynamicRiskCritic
launches must keep the same map, robot pose, speed limits, and scenario.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("predictive_nav_bringup")
    simulation_dir = get_package_share_directory("predictive_nav_simulation")

    default_map = os.path.join(
        simulation_dir, "maps", "dynamic_navigation_lab.yaml"
    )
    default_world = os.path.join(
        simulation_dir, "worlds", "dynamic_navigation_lab.sdf"
    )
    default_params = os.path.join(bringup_dir, "config", "nav2_dwb.yaml")
    default_rviz = os.path.join(bringup_dir, "rviz", "nav_baseline.rviz")

    # The simulation launch also has a ``use_rviz`` argument.  Keep its
    # forced-false value scoped to the include, otherwise it overwrites this
    # launch file's own ``use_rviz`` argument and prevents Nav2 RViz from
    # starting.
    simulation = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(simulation_dir, "launch", "dynamic_navigation_lab.launch.py")
            ),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "use_rviz": "false",
                # AMCL is the only publisher of map -> odom during known-map
                # navigation.  Do not also give odom a world parent.
                "use_world_odom_anchor": "false",
                "enable_scan_safety_guard": "true",
                "enable_dynamic_obstacle": LaunchConfiguration("enable_dynamic_obstacle"),
            }.items(),
        )],
    )

    common_parameters = [
        LaunchConfiguration("params_file"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]
    # Do not depend on nav2_bringup here.  Jazzy Debian's navigation2 meta
    # package provides the runtime nodes but not necessarily nav2_bringup.
    # Starting the lifecycle nodes explicitly also makes this baseline's
    # process set visible and stable for future benchmark instrumentation.
    nav2_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=common_parameters
            + [{"yaml_filename": LaunchConfiguration("map")}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=common_parameters,
        ),
    ]

    # Gazebo must publish /clock, /scan and odom→base_footprint before AMCL
    # configures.  Starting these managers immediately races the bridge and
    # makes AMCL intermittently fail its lifecycle configure transition.
    localization_manager = TimerAction(
        period=LaunchConfiguration("localization_start_delay"),
        actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": True,
                    "node_names": ["map_server", "amcl"],
                    "bond_timeout": 15.0,
                    "service_timeout": 20.0,
                    "bond_respawn_max_duration": 30.0,
                    "attempt_respawn_reconnection": False,
                }
            ],
        )],
    )
    navigation_manager = TimerAction(
        period=LaunchConfiguration("navigation_start_delay"),
        actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "smoother_server",
                        "behavior_server",
                        "bt_navigator",
                        "velocity_smoother",
                    ],
                    "bond_timeout": 15.0,
                    "service_timeout": 20.0,
                    "bond_respawn_max_duration": 30.0,
                    "attempt_respawn_reconnection": False,
                }
            ],
        )],
    )

    initial_pose = Node(
        package="predictive_nav_bringup",
        executable="publish_initial_pose.py",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {
                "x": LaunchConfiguration("spawn_x"),
                "y": LaunchConfiguration("spawn_y"),
                "yaw": LaunchConfiguration("spawn_yaw"),
                "publish_delay": LaunchConfiguration("initial_pose_delay"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav_baseline_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "enable_dynamic_obstacle",
                default_value="false",
                description="Enable the repeatable moving obstacle for dynamic-navigation trials.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="5.8"),
            DeclareLaunchArgument("spawn_y", default_value="-3.85"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
            DeclareLaunchArgument("initial_pose_delay", default_value="4.0"),
            DeclareLaunchArgument("localization_start_delay", default_value="8.0"),
            DeclareLaunchArgument("navigation_start_delay", default_value="12.0"),
            simulation,
            *nav2_nodes,
            localization_manager,
            navigation_manager,
            initial_pose,
            rviz,
        ]
    )
