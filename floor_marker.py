#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class FloorMarker(Node):
    def __init__(self):
        super().__init__('floor_marker')
        self.publisher_ = self.create_publisher(Marker, 'floor_marker', 10)
        
        # --- Marker 1: cobot1.dae ---
        self.marker1 = Marker()
        self.marker1.header.frame_id = "odom"
        self.marker1.ns = "AGV"
        self.marker1.id = 0
        self.marker1.type = Marker.MESH_RESOURCE
        self.marker1.action = Marker.ADD
        self.marker1.mesh_resource = "file:///home/effica/myagv_ros/src/my_agv_examples/cobot1.dae"
        self.marker1.mesh_use_embedded_materials = True

        # Position & orientation (your coordinates)
        self.marker1.pose.position.x = 0.0
        self.marker1.pose.position.y = 0.0
        self.marker1.pose.position.z = 0.0
        self.marker1.pose.orientation.x = 0.0
        self.marker1.pose.orientation.y = 0.0
        self.marker1.pose.orientation.z = 0.0
        self.marker1.pose.orientation.w = 10.0

        # Scale
        self.marker1.scale.x = 0.5
        self.marker1.scale.y = 0.5
        self.marker1.scale.z = 0.5

        # Color (visible even if no materials in dae)
        self.marker1.color.r = 1.0
        self.marker1.color.g = 1.0
        self.marker1.color.b = 1.0
        self.marker1.color.a = 1.0

        # --- Marker 2: cobot2.dae ---
        self.marker2 = Marker()
        self.marker2.header.frame_id = "odom"
        self.marker2.ns = "AGV2"
        self.marker2.id = 1
        self.marker2.type = Marker.MESH_RESOURCE
        self.marker2.action = Marker.ADD
        self.marker2.mesh_resource = "file:///home/effica/myagv_ros/src/my_agv_examples/COURSE_FLOOR_12.dae"
        self.marker2.mesh_use_embedded_materials = True

        # Same position & orientation as marker1
        self.marker2.pose.position.x = 0.0
        self.marker2.pose.position.y = 0.0
        self.marker2.pose.position.z = 0.0
        self.marker2.pose.orientation.x = 0.0
        self.marker2.pose.orientation.y = 0.0
        self.marker2.pose.orientation.z = 0.0
        self.marker2.pose.orientation.w = 10.0

        # Same scale
        self.marker2.scale.x = 0.5
        self.marker2.scale.y = 0.5
        self.marker2.scale.z = 0.5

        #colour
        self.marker2.color.r = 1.0
        self.marker2.color.g = 1.0
        self.marker2.color.b = 1.0
        self.marker2.color.a = 1.0

        # Timer to publish both
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.marker1.header.stamp = now
        self.marker2.header.stamp = now

        self.publisher_.publish(self.marker1)
        self.publisher_.publish(self.marker2)


def main(args=None):
    rclpy.init(args=args)
    node = FloorMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
