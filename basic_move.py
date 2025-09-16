#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BasicMove(Node):
    def __init__(self):
        super().__init__('basic_move')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run)  # run every 0.1 sec

        self.stage = 0
        self.start_time = time.time()
        self.get_logger().info("Starting simple path...")

    def run(self):
        twist = Twist()
        t = time.time() - self.start_time  # seconds passed

        if self.stage == 0:  # Move forward 5s (~5m if speed=1.0 m/s)
            twist.linear.x = 1.0
            if t > 3.0:
                self.next_stage()
        elif self.stage == 1:  # Turn right for 1.5s (~90 deg)
            twist.angular.z = -1.0
            if t > 1.5:
                self.next_stage()
        elif self.stage == 2:  # Move forward 5s
            twist.linear.x = 1.0
            if t > 3.0:
                self.next_stage()
        elif self.stage == 3:  # Move forward 5s
            twist.linear.x = 1.0
            if t > 3.0:
                self.next_stage()
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Finished path!")

        self.cmd_pub.publish(twist)

    def next_stage(self):
        self.stage += 1
        self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = BasicMove()  # ✔ This must match the class name
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
