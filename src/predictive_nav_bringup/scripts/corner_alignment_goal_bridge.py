#!/usr/bin/env python3
"""Turn RViz 2D Goal Pose clicks into corner-aware Nav2 navigation.

The RViz tool publishes a final goal on /corner_alignment/goal.  This node
asks Nav2 for one global path, inserts pre-alignment / post-turn poses at
sharp corners, and sends the resulting NavigateToPose goals sequentially.
It is an orchestration layer: it never publishes cmd_vel or changes any
controller, footprint, costmap, or safety parameter.
"""

import math
from typing import List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from corner_alignment_executor import (
    DetectedTurn,
    PathPoint,
    cumulative_lengths,
    detect_sharp_turns,
    path_points,
    sample_at_arc_length,
)


class CornerAlignmentGoalBridge(Node):
    """A non-blocking action state machine driven by RViz PoseStamped goals."""

    def __init__(self) -> None:
        super().__init__("corner_alignment_goal_bridge")
        self.declare_parameter("goal_topic", "/corner_alignment/goal")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("planner_action_name", "/compute_path_to_pose")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("heading_lookahead_m", 0.30)
        self.declare_parameter("turn_angle_threshold_rad", 0.80)
        self.declare_parameter("min_turn_separation_m", 0.80)
        self.declare_parameter("pre_alignment_distance_m", 0.70)
        self.declare_parameter("post_turn_distance_m", 0.35)
        self.declare_parameter("max_turns", 4)

        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.heading_lookahead_m = max(
            0.05, float(self.get_parameter("heading_lookahead_m").value)
        )
        self.turn_angle_threshold_rad = min(
            math.pi,
            max(0.05, float(self.get_parameter("turn_angle_threshold_rad").value)),
        )
        self.min_turn_separation_m = max(
            0.05, float(self.get_parameter("min_turn_separation_m").value)
        )
        self.pre_alignment_distance_m = max(
            self.heading_lookahead_m,
            float(self.get_parameter("pre_alignment_distance_m").value),
        )
        self.post_turn_distance_m = max(
            0.05, float(self.get_parameter("post_turn_distance_m").value)
        )
        self.max_turns = max(1, int(self.get_parameter("max_turns").value))

        goal_topic = str(self.get_parameter("goal_topic").value)
        planner_action_name = str(self.get_parameter("planner_action_name").value)
        navigate_action_name = str(self.get_parameter("navigate_action_name").value)
        self.goal_subscription = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10
        )
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, planner_action_name
        )
        self.navigate_client = ActionClient(self, NavigateToPose, navigate_action_name)

        self.busy = False
        self.final_goal: Optional[PoseStamped] = None
        self.stages: List[Tuple[str, PoseStamped]] = []
        self.stage_index = 0
        # A second RViz click does not run beside the old staged mission.  It is
        # kept as the latest pending goal while Nav2 cancels the one active
        # NavigateToPose action.  This avoids two independent action clients
        # competing to drive the robot.
        self.pending_goal: Optional[PoseStamped] = None
        self.preempt_requested = False
        self.active_compute_goal_handle = None
        self.active_navigation_goal_handle = None
        self.compute_request_in_progress = False
        self.navigation_request_in_progress = False
        self.navigation_cancel_in_progress = False
        self.mission_id = 0
        self.get_logger().info(
            f"Ready: click RViz 2D Goal Pose publishing to {goal_topic}."
        )

    def goal_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id != self.goal_frame:
            self.get_logger().error(
                f"Ignoring goal in frame '{message.header.frame_id}'; expected '{self.goal_frame}'."
            )
            return
        position = message.pose.position
        if not (math.isfinite(position.x) and math.isfinite(position.y)):
            self.get_logger().error("Ignoring a goal with non-finite x/y coordinates.")
            return
        if not self.compute_path_client.server_is_ready() or not self.navigate_client.server_is_ready():
            self.get_logger().warn(
                "Nav2 planner/navigation action server is not ready; wait for Nav2 active, then click again."
            )
            return

        if self.busy:
            self.request_preemption(message)
            return

        self.start_mission(message)

    def start_mission(self, message: PoseStamped) -> None:
        """Plan and execute one final RViz goal from the robot's current pose."""
        position = message.pose.position
        self.mission_id += 1
        mission_id = self.mission_id
        self.busy = True
        self.final_goal = message
        self.stages = []
        self.stage_index = 0
        self.pending_goal = None
        self.preempt_requested = False
        self.active_compute_goal_handle = None
        self.active_navigation_goal_handle = None
        self.compute_request_in_progress = True
        self.navigation_request_in_progress = False
        self.navigation_cancel_in_progress = False
        self.get_logger().info(
            f"Received corner-aware final goal: x={position.x:.2f}, y={position.y:.2f}."
        )
        request = ComputePathToPose.Goal()
        request.goal = message
        request.use_start = False
        future = self.compute_path_client.send_goal_async(request)
        future.add_done_callback(
            lambda result, current_id=mission_id: self.compute_goal_response_callback(
                result, current_id
            )
        )

    def request_preemption(self, message: PoseStamped) -> None:
        """Replace an active staged mission with the newest RViz click safely."""
        self.pending_goal = message
        self.preempt_requested = True
        position = message.pose.position
        self.get_logger().info(
            f"New goal received; preempting current mission for x={position.x:.2f}, "
            f"y={position.y:.2f}."
        )

        # Path computation cannot move the robot, so cancel/discard it and
        # immediately compute the newest path.  Navigation actions, however,
        # must first report their cancellation before the new first stage is
        # submitted.
        if self.active_compute_goal_handle is not None:
            self.active_compute_goal_handle.cancel_goal_async()
            self.start_pending_mission()
        elif self.active_navigation_goal_handle is not None:
            self.cancel_active_navigation_stage()
        else:
            self.get_logger().info(
                "Preemption queued while the current action request is being accepted."
            )

    def start_pending_mission(self) -> bool:
        if not self.preempt_requested or self.pending_goal is None:
            return False
        pending_goal = self.pending_goal
        self.get_logger().info(
            "Previous corner-aware mission stopped; replanning from the current robot pose."
        )
        self.clear_mission()
        self.start_mission(pending_goal)
        return True

    def cancel_active_navigation_stage(self) -> None:
        if (
            self.active_navigation_goal_handle is None
            or self.navigation_cancel_in_progress
        ):
            return
        self.navigation_cancel_in_progress = True
        self.get_logger().info("Cancelling the current navigation stage for the new goal.")
        future = self.active_navigation_goal_handle.cancel_goal_async()
        future.add_done_callback(self.navigation_cancel_response_callback)

    def navigation_cancel_response_callback(self, future) -> None:
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warn(
                    "Nav2 did not acknowledge stage cancellation yet; waiting for its action result."
                )
        except Exception as error:
            self.get_logger().warn(
                f"Could not request navigation-stage cancellation: {error}. Waiting for result."
            )

    def is_current_mission(self, mission_id: int) -> bool:
        return self.busy and mission_id == self.mission_id

    def compute_goal_response_callback(self, future, mission_id: int) -> None:
        if not self.is_current_mission(mission_id):
            return
        self.compute_request_in_progress = False
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy reports transport errors as exceptions.
            if self.start_pending_mission():
                return
            self.finish_with_error(f"Could not send ComputePathToPose: {error}")
            return
        if goal_handle is None or not goal_handle.accepted:
            if self.start_pending_mission():
                return
            self.finish_with_error("Planner rejected the corner-aware goal.")
            return
        self.active_compute_goal_handle = goal_handle
        if self.preempt_requested:
            goal_handle.cancel_goal_async()
            self.start_pending_mission()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, current_id=mission_id: self.compute_result_callback(
                result, current_id
            )
        )

    def compute_result_callback(self, future, mission_id: int) -> None:
        if not self.is_current_mission(mission_id):
            return
        self.active_compute_goal_handle = None
        try:
            wrapped_result = future.result()
        except Exception as error:
            if self.start_pending_mission():
                return
            self.finish_with_error(f"Could not receive ComputePathToPose result: {error}")
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            if self.start_pending_mission():
                return
            self.finish_with_error(
                f"ComputePathToPose ended with action status {wrapped_result.status}."
            )
            return

        result = wrapped_result.result
        if result.error_code != ComputePathToPose.Result.NONE:
            if self.start_pending_mission():
                return
            self.finish_with_error(
                f"Planner returned error code {result.error_code}."
            )
            return

        points = path_points(result.path.poses)
        if len(points) < 2:
            if self.start_pending_mission():
                return
            self.finish_with_error("Planner returned fewer than two finite path points.")
            return
        self.stages = self.build_stages(points)
        self.stage_index = 0
        self.get_logger().info(
            f"Global path has {len(points)} finite points; generated {len(self.stages)} stage(s)."
        )
        self.send_next_stage(mission_id)

    def build_stages(self, points: Sequence[PathPoint]) -> List[Tuple[str, PoseStamped]]:
        if self.final_goal is None:
            return []
        lengths = cumulative_lengths(points)
        turns: List[DetectedTurn] = detect_sharp_turns(
            points,
            lengths,
            self.heading_lookahead_m,
            self.turn_angle_threshold_rad,
            self.min_turn_separation_m,
            self.max_turns,
        )
        stages: List[Tuple[str, PoseStamped]] = []
        for index, turn in enumerate(turns, start=1):
            pre_arc_length_m = max(0.0, turn.arc_length_m - self.pre_alignment_distance_m)
            post_arc_length_m = min(
                lengths[-1], turn.arc_length_m + self.post_turn_distance_m
            )
            pre_point = sample_at_arc_length(points, lengths, pre_arc_length_m)
            post_point = sample_at_arc_length(points, lengths, post_arc_length_m)
            self.get_logger().info(
                f"Detected turn {index}: {math.degrees(turn.angle_rad):.1f} deg; "
                f"pre-align at ({pre_point.x:.2f}, {pre_point.y:.2f}), "
                f"exit at ({post_point.x:.2f}, {post_point.y:.2f})."
            )
            stages.append(
                (
                    f"turn {index} pre-alignment",
                    self.make_pose(pre_point, turn.outgoing_yaw_rad),
                )
            )
            stages.append(
                (
                    f"turn {index} post-turn exit",
                    self.make_pose(post_point, turn.outgoing_yaw_rad),
                )
            )
        stages.append(("final goal", self.final_goal))
        return stages

    def make_pose(self, point: PathPoint, yaw_rad: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
        return pose

    def send_next_stage(self, mission_id: int) -> None:
        if not self.is_current_mission(mission_id):
            return
        if self.preempt_requested:
            self.start_pending_mission()
            return
        if self.stage_index >= len(self.stages):
            self.get_logger().info("Corner-aware navigation completed successfully.")
            self.clear_mission()
            return
        name, pose = self.stages[self.stage_index]
        yaw_rad = 2.0 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        self.get_logger().info(
            f"Stage {self.stage_index + 1}/{len(self.stages)}: {name}: "
            f"x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, "
            f"yaw={yaw_rad:.2f} rad."
        )
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.navigation_request_in_progress = True
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, current_id=mission_id: self.navigate_goal_response_callback(
                result, current_id
            )
        )

    def navigate_goal_response_callback(self, future, mission_id: int) -> None:
        if not self.is_current_mission(mission_id):
            return
        self.navigation_request_in_progress = False
        try:
            goal_handle = future.result()
        except Exception as error:
            if self.start_pending_mission():
                return
            self.finish_with_error(f"Could not send navigation stage: {error}")
            return
        if goal_handle is None or not goal_handle.accepted:
            if self.start_pending_mission():
                return
            self.finish_with_error("Nav2 rejected a generated corner-aware stage.")
            return
        self.active_navigation_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, current_id=mission_id: self.navigate_result_callback(
                result, current_id
            )
        )
        if self.preempt_requested:
            self.cancel_active_navigation_stage()

    def navigate_result_callback(self, future, mission_id: int) -> None:
        if not self.is_current_mission(mission_id):
            return
        self.active_navigation_goal_handle = None
        self.navigation_cancel_in_progress = False
        try:
            wrapped_result = future.result()
        except Exception as error:
            if self.start_pending_mission():
                return
            self.finish_with_error(f"Could not receive navigation result: {error}")
            return
        if self.preempt_requested:
            self.start_pending_mission()
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = wrapped_result.result
            self.finish_with_error(
                f"Stage {self.stage_index + 1} failed with action status "
                f"{wrapped_result.status}, error={result.error_code}."
            )
            return
        self.stage_index += 1
        self.send_next_stage(mission_id)

    def finish_with_error(self, message: str) -> None:
        self.get_logger().error(message)
        self.clear_mission()

    def clear_mission(self) -> None:
        self.busy = False
        self.final_goal = None
        self.stages = []
        self.stage_index = 0
        self.active_compute_goal_handle = None
        self.active_navigation_goal_handle = None
        self.compute_request_in_progress = False
        self.navigation_request_in_progress = False
        self.navigation_cancel_in_progress = False


def main() -> None:
    rclpy.init()
    node = CornerAlignmentGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
