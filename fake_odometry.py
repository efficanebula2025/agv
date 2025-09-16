#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def yaw_to_quat_z(yaw: float):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))

class FakeOdometry(Node):
    def __init__(self):
        super().__init__('fake_odometry')

        # --- Parameters
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('heartbeat_sec', 5.0)  # log every N seconds (0 = off)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').get_parameter_value().double_value)

        # --- State (world)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Latest body-frame command
        self.vx = 0.0  # forward
        self.vy = 0.0  # strafe (left +, right -)
        self.wz = 0.0  # yaw rate

        # --- I/O
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, qos_cmd)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = TransformBroadcaster(self)

        # --- Timing
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(hz, 1.0), self.update)
        self._last_heartbeat = self.get_clock().now()  # for periodic log

        self.get_logger().info(
            f"fake_odometry: TF {self.frame_id} -> {self.child_frame_id} @ {hz:.1f} Hz"
        )

    def cmd_cb(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        # Clamp dt to keep it sane after pauses
        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / 50.0
        self.last_time = now

        # --- Integrate holonomic planar model
        self.yaw += self.wz * dt
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        vx_w = self.vx * cy - self.vy * sy
        vy_w = self.vx * sy + self.vy * cy
        self.x += vx_w * dt
        self.y += vy_w * dt

        qx, qy, qz, qw = yaw_to_quat_z(self.yaw)

        # --- Publish TF: frame_id -> child_frame_id
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)

        # --- Publish /odom
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.frame_id
        od.child_frame_id = self.child_frame_id
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.vx
        od.twist.twist.linear.y = self.vy
        od.twist.twist.angular.z = self.wz
        self.odom_pub.publish(od)

        # --- Heartbeat log (optional)
        if self.heartbeat_sec > 0.0:
            if (now - self._last_heartbeat).nanoseconds * 1e-9 >= self.heartbeat_sec:
                self.get_logger().info("Broadcasting TF odom -> base_footprint")
                self._last_heartbeat = now

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
