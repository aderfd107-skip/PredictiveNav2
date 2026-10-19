#!/usr/bin/env python3
"""Execute a Nav2 goal with path-geometry corner pre-alignment.

This is a deliberately small, behavior-layer prototype.  It asks Nav2's
global planner for a path, detects sharp path-heading changes, then sends
ordinary NavigateToPose goals at a safe distance before and after each turn.
It never publishes cmd_vel and it never changes DWB, costmap, or footprint
parameters.

V1 only reasons about global-path geometry.  It is appropriate for the known,
static Predictive Dynamic Lab.  A later V2 must validate every generated pose
and connecting segment against the live costmap before claiming generic use.
"""

import bisect
import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass(frozen=True)
class PathPoint:
    """A finite 2-D point sampled from Nav2's global path."""

    x: float
    y: float


@dataclass(frozen=True)
class DetectedTurn:
    """A sharp path-heading change represented by arc length and exit yaw."""

    arc_length_m: float
    outgoing_yaw_rad: float
    angle_rad: float


def normalize_angle(angle_rad: float) -> float:
    """Return an angle in [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def distance_2d(left: PathPoint, right: PathPoint) -> float:
    return math.hypot(right.x - left.x, right.y - left.y)


def interpolate(left: PathPoint, right: PathPoint, ratio: float) -> PathPoint:
    return PathPoint(
        x=left.x + (right.x - left.x) * ratio,
        y=left.y + (right.y - left.y) * ratio,
    )


def path_points(path_poses: Sequence[PoseStamped]) -> List[PathPoint]:
    """Read finite path positions and drop duplicate consecutive samples."""
    points: List[PathPoint] = []
    for pose_stamped in path_poses:
        point = PathPoint(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        if not (math.isfinite(point.x) and math.isfinite(point.y)):
            continue
        if not points or distance_2d(points[-1], point) > 1.0e-6:
            points.append(point)
    return points


def cumulative_lengths(points: Sequence[PathPoint]) -> List[float]:
    lengths = [0.0]
    for index in range(1, len(points)):
        lengths.append(lengths[-1] + distance_2d(points[index - 1], points[index]))
    return lengths


def sample_at_arc_length(
    points: Sequence[PathPoint], lengths: Sequence[float], arc_length_m: float
) -> PathPoint:
    """Linearly interpolate a point at a clamped distance along a polyline."""
    if not points:
        raise ValueError("Cannot sample an empty path.")
    target = min(max(arc_length_m, 0.0), lengths[-1])
    right_index = bisect.bisect_right(lengths, target)
    if right_index == 0:
        return points[0]
    if right_index >= len(points):
        return points[-1]

    left_index = right_index - 1
    segment_length = lengths[right_index] - lengths[left_index]
    if segment_length <= 1.0e-9:
        return points[right_index]
    ratio = (target - lengths[left_index]) / segment_length
    return interpolate(points[left_index], points[right_index], ratio)


def heading_between(left: PathPoint, right: PathPoint) -> float:
    return math.atan2(right.y - left.y, right.x - left.x)


def detect_sharp_turns(
    points: Sequence[PathPoint],
    lengths: Sequence[float],
    heading_lookahead_m: float,
    turn_angle_threshold_rad: float,
    min_turn_separation_m: float,
    max_turns: int,
) -> List[DetectedTurn]:
    """Return one strongest representative for every separated turn region."""
    if len(points) < 3 or not lengths or lengths[-1] < 2.0 * heading_lookahead_m:
        return []

    detected: List[DetectedTurn] = []
    region_best: Optional[DetectedTurn] = None

    def finish_region() -> None:
        nonlocal region_best
        if region_best is None:
            return
        if (
            not detected
            or region_best.arc_length_m - detected[-1].arc_length_m
            >= min_turn_separation_m
        ):
            detected.append(region_best)
        elif region_best.angle_rad > detected[-1].angle_rad:
            detected[-1] = region_best
        region_best = None

    for index, corner in enumerate(points):
        arc_length_m = lengths[index]
        if (
            arc_length_m < heading_lookahead_m
            or lengths[-1] - arc_length_m < heading_lookahead_m
        ):
            finish_region()
            continue

        before = sample_at_arc_length(
            points, lengths, arc_length_m - heading_lookahead_m
        )
        after = sample_at_arc_length(
            points, lengths, arc_length_m + heading_lookahead_m
        )
        incoming_yaw = heading_between(before, corner)
        outgoing_yaw = heading_between(corner, after)
        turn_angle_rad = abs(normalize_angle(outgoing_yaw - incoming_yaw))

        if turn_angle_rad >= turn_angle_threshold_rad:
            candidate = DetectedTurn(
                arc_length_m=arc_length_m,
                outgoing_yaw_rad=outgoing_yaw,
                angle_rad=turn_angle_rad,
            )
            if region_best is None or candidate.angle_rad > region_best.angle_rad:
                region_best = candidate
        else:
            finish_region()

    finish_region()
    return detected[:max_turns]


class CornerAlignmentExecutor(Node):
    """Compute a global plan, then execute corner-aware intermediate goals."""

    def __init__(self) -> None:
        super().__init__("corner_alignment_executor")
        self.declare_parameter("goal_x", math.nan)
        self.declare_parameter("goal_y", math.nan)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("planner_action_name", "/compute_path_to_pose")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("server_wait_sec", 20.0)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("heading_lookahead_m", 0.30)
        self.declare_parameter("turn_angle_threshold_rad", 0.80)
        self.declare_parameter("min_turn_separation_m", 0.80)
        self.declare_parameter("pre_alignment_distance_m", 0.70)
        self.declare_parameter("post_turn_distance_m", 0.35)
        self.declare_parameter("max_turns", 4)
        self.declare_parameter("dry_run", False)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.server_wait_sec = max(
            1.0, float(self.get_parameter("server_wait_sec").value)
        )
        self.goal_timeout_sec = max(
            1.0, float(self.get_parameter("goal_timeout_sec").value)
        )
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
        self.dry_run = bool(self.get_parameter("dry_run").value)

        planner_action_name = str(self.get_parameter("planner_action_name").value)
        navigate_action_name = str(self.get_parameter("navigate_action_name").value)
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, planner_action_name
        )
        self.navigate_client = ActionClient(self, NavigateToPose, navigate_action_name)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def wait_for_servers(self) -> bool:
        deadline = time.monotonic() + self.server_wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            planner_ready = self.compute_path_client.wait_for_server(timeout_sec=0.5)
            navigation_ready = self.navigate_client.wait_for_server(timeout_sec=0.5)
            if planner_ready and navigation_ready:
                return True
            self.get_logger().info("Waiting for planner and navigation action servers ...")
        return False

    def compute_path(self, final_goal: PoseStamped) -> Optional[List[PathPoint]]:
        goal = ComputePathToPose.Goal()
        goal.goal = final_goal
        goal.use_start = False
        send_future = self.compute_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.server_wait_sec)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Timed out while requesting the global path.")
            return None

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Planner rejected ComputePathToPose.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.server_wait_sec)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Timed out while waiting for the global path.")
            return None

        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"ComputePathToPose ended with action status {wrapped_result.status}."
            )
            return None

        result = wrapped_result.result
        if result.error_code != ComputePathToPose.Result.NONE:
            self.get_logger().error(
                f"Planner error {result.error_code}: {result.error_msg or 'no message'}"
            )
            return None

        points = path_points(result.path.poses)
        if len(points) < 2:
            self.get_logger().error("Planner returned fewer than two finite path points.")
            return None
        return points

    def build_stages(self, points: Sequence[PathPoint]) -> List[Tuple[str, PoseStamped]]:
        lengths = cumulative_lengths(points)
        turns = detect_sharp_turns(
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
            turn_degrees = math.degrees(turn.angle_rad)
            self.get_logger().info(
                f"Detected turn {index}: {turn_degrees:.1f} deg at "
                f"path arc {turn.arc_length_m:.2f} m; outgoing yaw="
                f"{turn.outgoing_yaw_rad:.2f} rad."
            )
            stages.append(
                (
                    f"turn {index} pre-alignment",
                    self.make_pose(pre_point.x, pre_point.y, turn.outgoing_yaw_rad),
                )
            )
            stages.append(
                (
                    f"turn {index} post-turn exit",
                    self.make_pose(post_point.x, post_point.y, turn.outgoing_yaw_rad),
                )
            )

        stages.append(("final goal", self.make_pose(self.goal_x, self.goal_y, self.goal_yaw)))
        return stages

    def cancel_goal(self, goal_handle) -> None:
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)

    def navigate_stage(self, stage_index: int, stage_count: int, name: str, pose: PoseStamped) -> bool:
        yaw = 2.0 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        self.get_logger().info(
            f"Stage {stage_index}/{stage_count}: {name}: x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}, yaw={yaw:.2f} rad"
        )
        if self.dry_run:
            return True

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.server_wait_sec)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Timed out while sending navigation goal.")
            return False
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.goal_timeout_sec)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Stage timed out; cancelling the active goal.")
            self.cancel_goal(goal_handle)
            return False

        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = wrapped_result.result
            self.get_logger().error(
                f"Stage failed with action status {wrapped_result.status}, "
                f"error={result.error_code}."
            )
            return False
        return True

    def run(self) -> bool:
        if not (math.isfinite(self.goal_x) and math.isfinite(self.goal_y)):
            self.get_logger().error("Set finite goal_x and goal_y parameters before running.")
            return False
        if not self.wait_for_servers():
            self.get_logger().error("Required Nav2 action servers did not become available.")
            return False

        final_goal = self.make_pose(self.goal_x, self.goal_y, self.goal_yaw)
        points = self.compute_path(final_goal)
        if points is None:
            return False
        stages = self.build_stages(points)
        self.get_logger().info(
            f"Global path has {len(points)} finite points; executing {len(stages)} stage(s)."
        )
        for index, (name, pose) in enumerate(stages, start=1):
            if not self.navigate_stage(index, len(stages), name, pose):
                self.get_logger().error("Stopping: no later stage will be sent.")
                return False
        self.get_logger().info("Corner-aware navigation completed successfully.")
        return True


def main() -> int:
    rclpy.init()
    node = CornerAlignmentExecutor()
    try:
        success = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Corner-aware navigation interrupted by user.")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
