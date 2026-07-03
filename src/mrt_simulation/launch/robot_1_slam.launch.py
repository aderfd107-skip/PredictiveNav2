import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    sim_dir = get_package_share_directory("mrt_simulation")
    office_launch = os.path.join(sim_dir, "launch", "office_service_mvp.launch.py")
    slam_params = os.path.join(sim_dir, "config", "robot_1_slam.yaml")
    rviz_config = os.path.join(sim_dir, "rviz", "robot_1_slam.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam_rviz = LaunchConfiguration("use_slam_rviz")
    autostart = LaunchConfiguration("autostart")
    slam_params_file = LaunchConfiguration("slam_params_file")

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[robot_1_slam] Activating slam_toolbox."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="robot_1_slam_rviz",
        output="screen",
        condition=IfCondition(use_slam_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time.",
            ),
            DeclareLaunchArgument(
                "use_slam_rviz",
                default_value="true",
                description="Start RViz with the robot_1 SLAM view.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically configure and activate slam_toolbox.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=slam_params,
                description="Path to slam_toolbox parameters for robot_1 mapping.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(office_launch),
                launch_arguments={
                    "use_rviz": "false",
                    "use_world_odom_anchors": "false",
                }.items(),
            ),
            slam_node,
            configure_slam,
            activate_slam,
            rviz_node,
        ]
    )
