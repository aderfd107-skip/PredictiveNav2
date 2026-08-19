#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSafetyGuard(Node):
    """Pass-through safety node.

    This node does NOT apply its own acceleration / jerk limiting — that
    is Gazebo DiffDrive's job.  Adding a second layer of rate-limiting
    here causes the two filters to fight each other, which produces
    lag, oscillation, and brief direction reversals during teleop.

    Responsibilities kept here:
      * laser-based speed scaling near obstacles
      * angular velocity is scaled together with linear so the robot
        does not pivot in place near walls (the main wobble source)
      * stale-command timeout (auto-stop when the pilot stops sending)
    """

    def __init__(self):
        super().__init__("scan_safety_guard")

        self.declare_parameter("stop_distance", 0.48)
        self.declare_parameter("slow_distance", 0.85)
        self.declare_parameter("front_sector_degrees", 46.0)
        self.declare_parameter("rear_sector_degrees", 42.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cmd_timeout", 0.35)
        self.declare_parameter("distance_smooth_alpha", 0.25)

        self.stop_distance = self.get_float_parameter("stop_distance")
        self.slow_distance = max(
            self.get_float_parameter("slow_distance"), self.stop_distance
        )
        self.front_sector = math.radians(
            self.get_float_parameter("front_sector_degrees")
        )
        self.rear_sector = math.radians(
            self.get_float_parameter("rear_sector_degrees")
        )
        self.cmd_timeout = self.get_float_parameter("cmd_timeout")
        self.distance_alpha = max(
            0.0, min(1.0, self.get_float_parameter("distance_smooth_alpha"))
        )

        publish_rate = max(self.get_float_parameter("publish_rate"), 1.0)
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.front_distance = math.inf
        self.rear_distance = math.inf

        self.publisher = self.create_publisher(Twist, "cmd_vel_out", 10)
        self.create_subscription(Twist, "cmd_vel_in", self.on_cmd, 10)
        self.create_subscription(
            LaserScan, "scan", self.on_scan, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_guarded_cmd)

        self.get_logger().info(
            "Scan safety guard active: stop <= %.2f m, slow <= %.2f m"
            % (self.stop_distance, self.slow_distance)
        )

    def get_float_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    # ----- subscriptions --------------------------------------------------

    def on_cmd(self, msg):
        self.last_cmd = copy.deepcopy(msg)
        self.last_cmd_time = self.get_clock().now()

    def on_scan(self, msg):
        front = []
        rear = []

        angle = msg.angle_min
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                normalized = math.atan2(math.sin(angle), math.cos(angle))
                if abs(normalized) <= self.front_sector * 0.5:
                    front.append(distance)
                elif abs(abs(normalized) - math.pi) <= self.rear_sector * 0.5:
                    rear.append(distance)
            angle += msg.angle_increment

        raw_front = min(front) if front else math.inf
        raw_rear = min(rear) if rear else math.inf

        # Exponential moving average so single-frame laser jitter or a wall
        # that briefly enters the sector during a turn does not cause speed
        # to oscillate.
        if self.distance_alpha <= 0.0:
            self.front_distance = raw_front
            self.rear_distance = raw_rear
        else:
            # Handle inf → finite transition (e.g. robot approaches a wall
            # that was previously out of range).
            if math.isfinite(self.front_distance) and math.isfinite(raw_front):
                self.front_distance = (
                    self.distance_alpha * raw_front
                    + (1.0 - self.distance_alpha) * self.front_distance
                )
            else:
                self.front_distance = raw_front

            if math.isfinite(self.rear_distance) and math.isfinite(raw_rear):
                self.rear_distance = (
                    self.distance_alpha * raw_rear
                    + (1.0 - self.distance_alpha) * self.rear_distance
                )
            else:
                self.rear_distance = raw_rear

    # ----- publisher -------------------------------------------------------

    def publish_guarded_cmd(self):
        target_cmd = copy.deepcopy(self.last_cmd)

        if self.command_is_stale():
            target_cmd = Twist()

        # Pass through directly — NO acceleration limiting.
        # Gazebo DiffDrive handles all velocity smoothing internally.
        cmd = copy.deepcopy(target_cmd)

        # Deadband: force near-zero commands to exactly zero so floating-
        # point noise cannot cause a spurious negative linear velocity
        # that Gazebo would then act on.
        if abs(cmd.linear.x) < 1e-4:
            cmd.linear.x = 0.0
        if abs(cmd.angular.z) < 1e-4:
            cmd.angular.z = 0.0

        if not self.command_is_stale():
            cmd = self._apply_laser_safety(cmd)

        # Hard clamp: if the pilot did not ask for reverse, never output
        # reverse — even if laser scaling or numeric noise would produce it.
        if target_cmd.linear.x >= 0.0 and cmd.linear.x < 0.0:
            cmd.linear.x = 0.0
        if target_cmd.linear.x <= 0.0 and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0

        self.publisher.publish(cmd)

    def _apply_laser_safety(self, cmd):
        """Scale velocities based on closest obstacle in each direction.

        Both linear *and* angular velocity are scaled by the same factor
        so the robot does not pivot in place when the laser briefly
        catches a side wall during a turn or in a narrow passage.
        """
        front_scale = self._speed_scale(self.front_distance)
        rear_scale = self._speed_scale(self.rear_distance)

        if cmd.linear.x > 0.0:
            cmd.linear.x *= front_scale
        elif cmd.linear.x < 0.0:
            cmd.linear.x *= rear_scale

        # Scale angular velocity together with linear so near-wall
        # slowdown is smooth rather than pivoting.
        ang_scale = min(front_scale, rear_scale)
        cmd.angular.z *= ang_scale

        return cmd

    def _speed_scale(self, distance):
        """Return 0.0 – 1.0 multiplier for a given obstacle distance."""
        if distance <= self.stop_distance:
            return 0.0
        if distance >= self.slow_distance:
            return 1.0
        return (distance - self.stop_distance) / (
            self.slow_distance - self.stop_distance
        )

    # ----- helpers ----------------------------------------------------------

    def command_is_stale(self):
        if self.last_cmd_time is None:
            return True
        if self.cmd_timeout <= 0.0:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age > self.cmd_timeout


def main():
    rclpy.init()
    node = ScanSafetyGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
