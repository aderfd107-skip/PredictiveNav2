#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class CollisionGuard(Node):
    def __init__(self):
        super().__init__("collision_guard")

        self.declare_parameter("stop_distance", 0.48)
        self.declare_parameter("slow_distance", 0.85)
        self.declare_parameter("front_sector_degrees", 46.0)
        self.declare_parameter("rear_sector_degrees", 42.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cmd_timeout", 0.35)

        self.stop_distance = self.get_float_parameter("stop_distance")
        self.slow_distance = max(
            self.get_float_parameter("slow_distance"), self.stop_distance
        )
        self.front_sector = math.radians(
            self.get_float_parameter("front_sector_degrees")
        )
        self.rear_sector = math.radians(self.get_float_parameter("rear_sector_degrees"))
        self.cmd_timeout = self.get_float_parameter("cmd_timeout")

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
            "Collision guard active: stop <= %.2f m, slow <= %.2f m"
            % (self.stop_distance, self.slow_distance)
        )

    def get_float_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

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

        self.front_distance = min(front) if front else math.inf
        self.rear_distance = min(rear) if rear else math.inf

    def publish_guarded_cmd(self):
        cmd = copy.deepcopy(self.last_cmd)

        if self.command_is_stale():
            cmd = Twist()
        else:
            cmd.linear.x = self.limit_axis(cmd.linear.x, self.front_distance)
            cmd.linear.x = -self.limit_axis(-cmd.linear.x, self.rear_distance)

            if cmd.linear.x == 0.0 and abs(self.last_cmd.linear.x) > 1e-6:
                cmd.linear.y = 0.0

        self.publisher.publish(cmd)

    def command_is_stale(self):
        if self.last_cmd_time is None:
            return True
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age > self.cmd_timeout

    def limit_axis(self, velocity, distance):
        if velocity <= 0.0:
            return velocity
        if distance <= self.stop_distance:
            return 0.0
        if distance >= self.slow_distance:
            return velocity

        scale = (distance - self.stop_distance) / (
            self.slow_distance - self.stop_distance
        )
        return velocity * max(0.0, min(1.0, scale))


def main():
    rclpy.init()
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
