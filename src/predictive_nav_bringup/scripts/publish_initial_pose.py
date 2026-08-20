#!/usr/bin/env python3
"""Publish a known simulation spawn pose only after localization inputs exist."""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")

        self.declare_parameter("x", 5.8)
        self.declare_parameter("y", -3.85)
        self.declare_parameter("yaw", 1.5708)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_delay", 4.0)
        self.declare_parameter("publish_period", 2.0)
        self.declare_parameter("max_attempts", 30)
        self.declare_parameter("xy_tolerance", 0.20)
        self.declare_parameter("yaw_tolerance", 0.35)
        self.declare_parameter("required_stable_count", 3)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        self.target_x = float(self.get_parameter("x").value)
        self.target_y = float(self.get_parameter("y").value)
        self.target_yaw = float(self.get_parameter("yaw").value)
        self.publish_delay = max(0.0, float(self.get_parameter("publish_delay").value))
        self.publish_period = max(0.5, float(self.get_parameter("publish_period").value))
        self.max_attempts = max(1, int(self.get_parameter("max_attempts").value))
        self.xy_tolerance = max(0.01, float(self.get_parameter("xy_tolerance").value))
        self.yaw_tolerance = max(0.01, float(self.get_parameter("yaw_tolerance").value))
        self.required_stable_count = max(
            1, int(self.get_parameter("required_stable_count").value)
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, map_qos
        )
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self._on_scan, scan_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("amcl_pose_topic").value,
            self._on_amcl_pose,
            10,
        )

        self.map_received = False
        self.scan_received = False
        self.delay_elapsed = False
        self.publish_started = False
        self.initial_pose_confirmed = False
        self.attempt_count = 0
        self.stable_alignment_count = 0
        self.publish_timer = None
        self.delay_timer = None
        if self.publish_delay == 0.0:
            self.delay_elapsed = True
        else:
            self.delay_timer = self.create_timer(
                self.publish_delay, self._on_delay_elapsed
            )
        self.readiness_timer = self.create_timer(0.5, self._try_start_publishing)

        self.get_logger().info(
            "Waiting for /map and /scan before setting AMCL initial pose at "
            f"({self.target_x:.2f}, {self.target_y:.2f}, {self.target_yaw:.2f} rad)."
        )

    def _on_map(self, _):
        self.map_received = True

    def _on_scan(self, _):
        self.scan_received = True

    def _on_delay_elapsed(self):
        self.delay_elapsed = True
        if self.delay_timer is not None:
            self.delay_timer.cancel()

    def _try_start_publishing(self):
        if self.publish_started or not self.delay_elapsed:
            return
        if not self.map_received or not self.scan_received:
            return
        self.publish_started = True
        self._publish_initial_pose()
        self.publish_timer = self.create_timer(self.publish_period, self._publish_initial_pose)

    def _publish_initial_pose(self):
        if self.initial_pose_confirmed:
            return
        if self.attempt_count >= self.max_attempts:
            self.get_logger().warn("AMCL did not align before the initial-pose retry limit.")
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            return

        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.get_parameter("frame_id").value
        # A zero stamp requests the latest available odom transform in AMCL.
        # With simulation time, now() can be a few milliseconds newer than
        # the bridged odometry message and otherwise causes extrapolation
        # warnings during initial localization.
        message.pose.pose.position.x = self.target_x
        message.pose.pose.position.y = self.target_y
        message.pose.pose.orientation.z = math.sin(self.target_yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(self.target_yaw * 0.5)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891945200942
        self.pose_pub.publish(message)
        self.attempt_count += 1
        self.get_logger().info(
            f"Published AMCL initial pose ({self.attempt_count}/{self.max_attempts})."
        )

    def _on_amcl_pose(self, message):
        if self.initial_pose_confirmed or not self.publish_started:
            return
        position = message.pose.pose.position
        distance = math.hypot(position.x - self.target_x, position.y - self.target_y)
        orientation = message.pose.pose.orientation
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        yaw_error = abs(math.atan2(math.sin(yaw - self.target_yaw), math.cos(yaw - self.target_yaw)))
        if distance > self.xy_tolerance or yaw_error > self.yaw_tolerance:
            self.stable_alignment_count = 0
            return

        self.stable_alignment_count += 1
        if self.stable_alignment_count < self.required_stable_count:
            return
        self.initial_pose_confirmed = True
        if self.publish_timer is not None:
            self.publish_timer.cancel()
        self.readiness_timer.cancel()
        self.get_logger().info("AMCL initial pose alignment confirmed.")


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
