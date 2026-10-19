#!/usr/bin/env python3
"""Run a safe, repeatable staged mission through the 0.44 m lab outlet.

This is deliberately *not* a controller tuning tool.  It sends a sequence of
ordinary ``NavigateToPose`` goals, waiting for every goal to succeed before
the next one is sent.  Therefore it reproduces the manual, staged procedure
that can pass the outlet without reducing Nav2's collision radius or forcing
forward velocity.
"""

import math
import sys
import time
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass(frozen=True)
class Waypoint:
    """A named map-frame pose used as one independently safe Nav2 goal."""

    name: str
    x: float
    y: float
    yaw: float


# The outlet is between the divider inner face x=-2.94 and the west end of
# cross_north_west x=-2.50.  With robot_radius=0.135 m, x=-2.72 is the
# geometric centre line.  The points immediately before and after the slot
# are kept away from the horizontal wall's y=[5.25, 5.37] footprint.
#
# Do not ask DWB to make the 90-degree heading change beside either convex
# corner at (-2.50, 5.25)/(-2.50, 5.37).  A differential-drive robot cannot
# translate sideways: it needs to turn while there is still clear space on
# the approach side, then traverse the slot approximately straight.
ROUTES = {
    "northbound_outlet": (
        Waypoint("south approach", -1.80, 4.55, math.pi),
        Waypoint("south pre-alignment", -2.72, 4.55, math.pi / 2.0),
        Waypoint("south slot alignment", -2.72, 4.95, math.pi / 2.0),
        Waypoint("north slot exit", -2.72, 5.58, math.pi / 2.0),
        Waypoint("north departure", -1.70, 5.65, 0.0),
    ),
    "southbound_outlet": (
        Waypoint("north approach", -1.70, 6.07, math.pi),
        Waypoint("north pre-alignment", -2.72, 6.07, -math.pi / 2.0),
        Waypoint("north slot alignment", -2.72, 5.58, -math.pi / 2.0),
        Waypoint("south slot exit", -2.72, 4.95, -math.pi / 2.0),
        Waypoint("south departure", -1.80, 4.75, 0.0),
    ),
}


class NarrowPassageMission(Node):
    def __init__(self):
        super().__init__("narrow_passage_mission")
        self.declare_parameter("route", "northbound_outlet")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("server_wait_sec", 20.0)
        self.declare_parameter("goal_timeout_sec", 90.0)

        self.route_name = str(self.get_parameter("route").value)
        self.server_wait_sec = max(
            1.0, float(self.get_parameter("server_wait_sec").value)
        )
        self.goal_timeout_sec = max(
            1.0, float(self.get_parameter("goal_timeout_sec").value)
        )
        action_name = str(self.get_parameter("action_name").value)
        self.client = ActionClient(self, NavigateToPose, action_name)

    def make_goal(self, waypoint: Waypoint) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def wait_for_server(self) -> bool:
        deadline = time.monotonic() + self.server_wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.client.wait_for_server(timeout_sec=1.0):
                return True
            self.get_logger().info("Waiting for /navigate_to_pose action server ...")
        return False

    def cancel_goal(self, goal_handle) -> None:
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)

    def run_waypoint(self, waypoint: Waypoint, index: int, total: int) -> bool:
        self.get_logger().info(
            f"Stage {index}/{total}: {waypoint.name}: "
            f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.2f} rad"
        )
        send_future = self.client.send_goal_async(self.make_goal(waypoint))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.server_wait_sec)
        if not send_future.done():
            self.get_logger().error("Timed out while sending goal; no later stage will run.")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected; no later stage will run.")
            return False

        started = time.monotonic()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.goal_timeout_sec
        )
        elapsed = time.monotonic() - started
        if not result_future.done():
            self.get_logger().error(
                f"Stage timed out after {elapsed:.1f} s; cancelling it. "
                "No later stage will run."
            )
            self.cancel_goal(goal_handle)
            return False

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Stage ended with action status {result.status} after {elapsed:.1f} s; "
                "no later stage will run."
            )
            return False

        self.get_logger().info(f"Stage succeeded in {elapsed:.1f} s.")
        return True

    def run(self) -> bool:
        route = ROUTES.get(self.route_name)
        if route is None:
            available = ", ".join(sorted(ROUTES))
            self.get_logger().error(
                f"Unknown route '{self.route_name}'. Available routes: {available}."
            )
            return False

        if not self.wait_for_server():
            self.get_logger().error("/navigate_to_pose action server did not become available.")
            return False

        self.get_logger().info(
            f"Starting '{self.route_name}' with {len(route)} staged Nav2 goals. "
            "DWB and costmap parameters are unchanged."
        )
        for index, waypoint in enumerate(route, start=1):
            if not self.run_waypoint(waypoint, index, len(route)):
                return False

        self.get_logger().info("Narrow-passage mission succeeded.")
        return True


def main():
    rclpy.init()
    mission = NarrowPassageMission()
    try:
        success = mission.run()
    except KeyboardInterrupt:
        mission.get_logger().warn("Mission interrupted by user.")
        success = False
    finally:
        mission.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
