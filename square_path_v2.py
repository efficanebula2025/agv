#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MecanumSteps(Node):
    def __init__(self):
        super().__init__('mecanum_steps')

        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #  Step-by-step sequence (students can edit here)
        # type = "forward", "backward", "left", "right", "rotate_left", "rotate_right"
        self.steps = [
            {"type": "forward",      "distance": 1.0, "speed": 0.2},   # Step 1
            {"type": "right",        "distance": 1.0, "speed": 0.2},   # Step 2
            {"type": "backward",     "distance": 1.0, "speed": 0.2},   # Step 3
            {"type": "left",         "distance": 1.0, "speed": 0.2},   # Step 4
            {"type": "rotate_left",  "angle": 90.0,   "speed": 0.3},   # Step 5
            {"type": "rotate_right", "angle": 90.0,   "speed": 0.3},   # Step 6
        ] 

        # Precompute durations
        for step in self.steps:
            if step["type"] in ["forward", "backward", "left", "right"]:
                step["duration"] = step["distance"] / max(step["speed"], 1e-6)
            elif step["type"] in ["rotate_left", "rotate_right"]:
                rad = math.radians(step["angle"])
                step["duration"] = rad / max(step["speed"], 1e-6)

        # State machine
        self.current_step = 0
        self.step_start = self.get_clock().now()

        # Timer
        self.timer = self.create_timer(1.0 / 20.0, self.loop)  # 20 Hz

        self.get_logger().info("MecanumSteps started. Executing sequence…")

    def loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.step_start).nanoseconds * 1e-9
        cmd = Twist()

        if self.current_step < len(self.steps):
            step = self.steps[self.current_step]

            if step["type"] == "forward":
                if elapsed < step["duration"]:
                    cmd.linear.x = step["speed"]
                else:
                    self.next_step()

            elif step["type"] == "backward":
                if elapsed < step["duration"]:
                    cmd.linear.x = -step["speed"]
                else:
                    self.next_step()

            elif step["type"] == "right":
                if elapsed < step["duration"]:
                    cmd.linear.y = -step["speed"]   # right is negative Y
                else:
                    self.next_step()

            elif step["type"] == "left":
                if elapsed < step["duration"]:
                    cmd.linear.y = step["speed"]    # left is positive Y
                else:
                    self.next_step()

            elif step["type"] == "rotate_left":
                if elapsed < step["duration"]:
                    cmd.angular.z = step["speed"]
                else:
                    self.next_step()

            elif step["type"] == "rotate_right":
                if elapsed < step["duration"]:
                    cmd.angular.z = -step["speed"]
                else:
                    self.next_step()

        else:
            # Stop when sequence is done
            cmd = Twist()

        self.pub.publish(cmd)

    def next_step(self):
        self.current_step += 1
        self.step_start = self.get_clock().now()
        if self.current_step < len(self.steps):
            self.get_logger().info(
                f"Starting step {self.current_step+1}/{len(self.steps)}: {self.steps[self.current_step]}"
            )
        else:
            self.get_logger().info("Sequence complete")


def main(args=None):
    rclpy.init(args=args)
    node = MecanumSteps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop robot
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
