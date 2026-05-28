#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def values(text, default=None):
    if text is None:
        return default or []
    return [float(value) for value in text.split()]


def color_from_material(visual):
    material = visual.find("material")
    if material is None:
        return 0.62, 0.64, 0.66, 1.0

    color_text = None
    ambient = material.find("ambient")
    diffuse = material.find("diffuse")
    if diffuse is not None and diffuse.text:
        color_text = diffuse.text
    elif ambient is not None and ambient.text:
        color_text = ambient.text

    rgba = values(color_text, [0.62, 0.64, 0.66, 1.0])
    while len(rgba) < 4:
        rgba.append(1.0)
    return rgba[:4]


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class OfficeMapMarkerPublisher(Node):
    def __init__(self):
        super().__init__("office_map_marker_publisher")
        default_world = (
            get_package_share_directory("mrt_simulation")
            + "/worlds/office_service_mvp.sdf"
        )
        self.declare_parameter("world_file", default_world)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, "office_service_map", qos)

        world_file = self.get_parameter("world_file").get_parameter_value().string_value
        self.markers = self.load_markers(world_file)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()

    def load_markers(self, world_file):
        root = ET.parse(world_file).getroot()
        marker_array = MarkerArray()
        marker_id = 0

        for model in root.findall(".//world/model"):
            if model.findtext("static", "false").strip().lower() != "true":
                continue

            model_name = model.get("name", "model")
            pose = values(model.findtext("pose"), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            while len(pose) < 6:
                pose.append(0.0)

            if model_name == "ground_plane":
                marker_array.markers.append(
                    self.make_box_marker(
                        marker_id,
                        model_name,
                        [0.0, 0.0, -0.01, 0.0, 0.0, 0.0],
                        [14.5, 10.5, 0.02],
                        [0.26, 0.27, 0.29, 0.45],
                    )
                )
                marker_id += 1
                continue

            for visual in model.findall(".//visual"):
                geometry = visual.find("geometry")
                if geometry is None:
                    continue

                visual_pose = values(
                    visual.findtext("pose"), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                )
                while len(visual_pose) < 6:
                    visual_pose.append(0.0)

                world_pose = [
                    pose[0] + visual_pose[0],
                    pose[1] + visual_pose[1],
                    pose[2] + visual_pose[2],
                    pose[3] + visual_pose[3],
                    pose[4] + visual_pose[4],
                    pose[5] + visual_pose[5],
                ]
                color = color_from_material(visual)

                box = geometry.find("box")
                if box is not None:
                    size = values(box.findtext("size"), [1.0, 1.0, 1.0])
                    marker_array.markers.append(
                        self.make_box_marker(
                            marker_id,
                            model_name,
                            world_pose,
                            size,
                            color,
                        )
                    )
                    marker_id += 1
                    marker_array.markers.append(
                        self.make_box_outline_marker(
                            marker_id, f"{model_name}_outline", world_pose, size
                        )
                    )
                    marker_id += 1
                    continue

                cylinder = geometry.find("cylinder")
                if cylinder is not None:
                    radius = float(cylinder.findtext("radius", "0.1"))
                    length = float(cylinder.findtext("length", "0.05"))
                    marker_array.markers.append(
                        self.make_cylinder_marker(
                            marker_id, model_name, world_pose, radius, length, color
                        )
                    )
                    marker_id += 1

        self.get_logger().info(
            f"Loaded {len(marker_array.markers)} RViz map markers from {world_file}"
        )
        return marker_array

    def make_box_marker(self, marker_id, name, pose, size, color):
        marker = self.base_marker(marker_id, name, pose, color)
        marker.type = Marker.CUBE
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        return marker

    def make_box_outline_marker(self, marker_id, name, pose, size):
        marker = self.base_marker(marker_id, name, pose, [0.95, 0.94, 0.82, 1.0])
        marker.type = Marker.LINE_LIST
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035

        half_x = size[0] * 0.5
        half_y = size[1] * 0.5
        corners = [
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        ]
        yaw = pose[5]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_corners = []
        for x, y in corners:
            world_corners.append(
                (
                    pose[0] + x * cos_yaw - y * sin_yaw,
                    pose[1] + x * sin_yaw + y * cos_yaw,
                )
            )

        for start_index, end_index in ((0, 1), (1, 2), (2, 3), (3, 0)):
            marker.points.append(self.point(*world_corners[start_index], 0.045))
            marker.points.append(self.point(*world_corners[end_index], 0.045))
        return marker

    def make_cylinder_marker(self, marker_id, name, pose, radius, length, color):
        marker = self.base_marker(marker_id, name, pose, color)
        marker.type = Marker.CYLINDER
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = length
        return marker

    def base_marker(self, marker_id, name, pose, color):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = name
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0).to_msg()
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        marker.pose.orientation.z, marker.pose.orientation.w = quaternion_from_yaw(
            pose[5]
        )
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = max(color[3], 0.35)
        return marker

    def point(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main():
    rclpy.init()
    node = OfficeMapMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
