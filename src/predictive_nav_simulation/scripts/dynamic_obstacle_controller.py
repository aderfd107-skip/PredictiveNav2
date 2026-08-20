#!/usr/bin/env python3
"""Repeatable kinematic actor for dynamic-navigation experiments.

The saved map stays static.  This node moves one Gazebo model through the
bridged ``/world/.../set_pose`` service, which makes the actor observable to
the robot LiDAR and its local costmap.  It is intentionally a deterministic
test fixture, not a human-behaviour simulator.
"""

import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class DynamicObstacleController(Node):
    """Move ``dynamic_obstacle_actor`` back and forth along a configured line."""

    def __init__(self):
        super().__init__("dynamic_obstacle_controller")

        self.declare_parameter("start_x", 2.25)
        self.declare_parameter("start_y", -3.70)
        self.declare_parameter("end_x", 2.25)
        self.declare_parameter("end_y", -2.00)
        self.declare_parameter("speed", 0.35)
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("start_on_launch", True)
        self.declare_parameter("actor_name", "dynamic_obstacle_actor")

        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)
        self.end_x = float(self.get_parameter("end_x").value)
        self.end_y = float(self.get_parameter("end_y").value)
        self.speed = max(0.01, float(self.get_parameter("speed").value))
        self.loop = bool(self.get_parameter("loop").value)
        self.actor_name = str(self.get_parameter("actor_name").value)
        update_rate = max(1.0, float(self.get_parameter("update_rate").value))

        self.dx = self.end_x - self.start_x
        self.dy = self.end_y - self.start_y
        self.path_length = math.hypot(self.dx, self.dy)
        if self.path_length < 0.01:
            raise ValueError("Dynamic obstacle start and end points must differ")

        self.progress = 0.0
        self.direction = 1.0
        self.running = bool(self.get_parameter("start_on_launch").value)
        self.last_time = None
        self.service_ready_reported = False

        self.pose_client = self.create_client(
            SetEntityPose, "/world/dynamic_navigation_lab/set_pose"
        )
        self.ground_truth_publisher = self.create_publisher(
            Marker, "/dynamic_obstacle/ground_truth_marker", 10
        )
        self.clear_old_debug_label()
        self.start_service = self.create_service(
            Trigger, "~/start", self.start_callback
        )
        self.stop_service = self.create_service(
            Trigger, "~/stop", self.stop_callback
        )
        self.reset_service = self.create_service(
            Trigger, "~/reset", self.reset_callback
        )
        self.timer = self.create_timer(1.0 / update_rate, self.update)

        self.get_logger().info(
            "Dynamic actor '%s': (%.2f, %.2f) -> (%.2f, %.2f), %.2f m/s. "
            "Services: %s/start, /stop, /reset"
            % (self.actor_name, self.start_x, self.start_y, self.end_x, self.end_y,
               self.speed, self.get_fully_qualified_name())
        )

    def start_callback(self, _request, response):
        self.running = True
        self.last_time = None
        response.success = True
        response.message = "Dynamic obstacle started"
        return response

    def stop_callback(self, _request, response):
        self.running = False
        response.success = True
        response.message = "Dynamic obstacle stopped at its current pose"
        return response

    def reset_callback(self, _request, response):
        self.progress = 0.0
        self.direction = 1.0
        self.last_time = None
        self.send_pose()
        response.success = True
        response.message = "Dynamic obstacle reset to route start"
        return response

    def update(self):
        if not self.pose_client.service_is_ready():
            if not self.service_ready_reported:
                self.get_logger().info("Waiting for Gazebo set_pose service...")
            return

        if not self.service_ready_reported:
            self.service_ready_reported = True
            self.get_logger().info("Connected to Gazebo set_pose service")
            self.send_pose()

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        elapsed = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if not self.running or elapsed <= 0.0:
            return

        self.progress += self.direction * self.speed * elapsed / self.path_length
        if self.progress >= 1.0 or self.progress <= 0.0:
            if self.loop:
                self.progress = min(1.0, max(0.0, self.progress))
                self.direction *= -1.0
            else:
                self.progress = min(1.0, max(0.0, self.progress))
                self.running = False
        self.send_pose()

    def send_pose(self):
        x = self.start_x + self.progress * self.dx
        y = self.start_y + self.progress * self.dy
        yaw = math.atan2(self.direction * self.dy, self.direction * self.dx)

        # This is a Gazebo ground-truth overlay for experiment debugging only.
        # Later perception nodes must use /scan and publish a distinct result;
        # they must never consume this topic as an algorithm input.
        self.publish_ground_truth_marker(x, y, yaw)

        if not self.pose_client.service_is_ready():
            return

        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = self.actor_name
        request.entity.type = Entity.MODEL
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = 0.475
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_client.call_async(request)

    def publish_ground_truth_marker(self, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gazebo_ground_truth_debug/" + self.actor_name
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.475
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.95
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.0
        marker.color.a = 0.35
        self.ground_truth_publisher.publish(marker)

    def clear_old_debug_label(self):
        """Remove the text marker emitted by the initial debug-overlay version."""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.ns = "gazebo_ground_truth_debug"
        marker.id = 1
        marker.action = Marker.DELETE
        self.ground_truth_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 2 may raise KeyboardInterrupt again while destroying services
        # after launch has already delivered SIGINT to the process group.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
