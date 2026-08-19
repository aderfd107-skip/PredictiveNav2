import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    sim_dir = get_package_share_directory("predictive_nav_simulation")
    simulation_launch = os.path.join(sim_dir, "launch", "dynamic_navigation_lab.launch.py")
    mapping_params = os.path.join(sim_dir, "config", "mapping.yaml")
    rviz_config = os.path.join(sim_dir, "rviz", "mapping.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    slam_node = LifecycleNode(
        package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", namespace="",
        output="screen", parameters=[LaunchConfiguration("mapping_params_file"),
                                      {"use_sim_time": use_sim_time}],
    )
    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam_node),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart),
    )
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_node, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
    ))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_mapping_rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("mapping_params_file", default_value=mapping_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={"use_rviz": "false", "use_world_odom_anchor": "false"}.items(),
        ),
        slam_node,
        configure,
        activate,
        Node(package="rviz2", executable="rviz2", name="mapping_rviz", output="screen",
             condition=IfCondition(LaunchConfiguration("use_mapping_rviz")),
             arguments=["-d", rviz_config], parameters=[{"use_sim_time": use_sim_time}]),
    ])
