#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquarePath(Node):
    def __init__(self):
        super().__init__('square_path')

        # Parameters
        self.declare_parameter('side_length', 1.0)   # meters
        self.declare_parameter('linear_speed', 0.2)  # m/s
        self.declare_parameter('angular_speed', 0.3) # rad/s

        self.side_length = float(self.get_parameter('side_length').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        # Durations
        self.straight_duration = self.side_length / max(self.linear_speed, 1e-6)
        self.turn_duration = (math.pi/2.0) / max(self.angular_speed, 1e-6)

        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State machine
        self.state = 'straight'
        self.state_start = self.get_clock().now()
        self.sides_done = 0

        self.timer = self.create_timer(1.0/20.0, self.loop)  # 20 Hz

        self.get_logger().info(
            f"SquarePath: side={self.side_length}m, v={self.linear_speed} m/s, w={self.angular_speed} rad/s"
        )

    def loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds * 1e-9
        cmd = Twist()

        if self.state == 'straight':
            if elapsed < self.straight_duration:
                cmd.linear.x = self.linear_speed
            else:
                self.state = 'turn'
                self.state_start = now
                self.get_logger().info(f'Completed side {self.sides_done+1}/4 → turning 90°')
        elif self.state == 'turn':
            if elapsed < self.turn_duration:
                cmd.angular.z = self.angular_speed
            else:
                self.sides_done = (self.sides_done + 1) % 4
                self.state = 'straight'
                self.state_start = now
                if self.sides_done == 0:
                    self.get_logger().info('Square complete. Repeating…')

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
