#!/usr/bin/env python3
import sys, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

if sys.platform == 'win32':
    import msvcrt
else:
    import termios, tty
from select import select

HELP = """
Keyboard teleop (ROS2)
----------------------
Moving:
   q   w   e
   a   s   d
   z   x   c

anything else : stop

i/o : increase/decrease max speeds by 10%
k/l : increase/decrease only linear speed by 10%
m/, : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0,  0),
    'e': (0, 0, 0, -1),
    'a': (0, 1, 0,  0),
    'd': (0,-1, 0,  0),
    'q': (0, 0, 0,  1),
    'x': (-1,0, 0,  0),
    'c': (-1,0, 0, -1),
    'z': (-1,0, 0,  1),

    'I': (1, 0, 0,  0),
    'O': (0, 0, 0, -1),
    'J': (0, 1, 0,  0),
    'L': (0,-1, 0,  0),
    'U': (0, 0, 0,  1),
    '<': (-1,0, 0,  0),
    '>': (-1,0, 0, -1),
    'M': (-1,0, 0,  1),
}

speedBindings = {
    'i': (1.1, 1.1),
    'o': (0.9, 0.9),
    'k': (1.1, 1.0),
    'l': (0.9, 1.0),
    'm': (1.0, 1.1),
    ',': (1.0, 0.9),
}

def save_terminal():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restore_terminal(old):
    if sys.platform == 'win32' or old is None:
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

def getKey(timeout):
    if sys.platform == 'win32':
        return msvcrt.getwch() if msvcrt.kbhit() else ''
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if r else ''
    term = save_terminal()
    restore_terminal(term)
    return key

class PublisherThread(threading.Thread):
    def __init__(self, node: Node, topic: str, rate_hz: float, stamped: bool, frame_id: str):
        super().__init__(daemon=True)
        self.node = node
        self.pub = node.create_publisher(TwistStamped if stamped else Twist, topic, 10)
        self.x = self.y = self.z = self.th = 0.0
        self.speed = 0.25
        self.turn  = 0.5
        self.stamped = stamped
        self.frame_id = frame_id
        self.done = False
        self.cond = threading.Condition()
        self.timeout = None if rate_hz == 0.0 else (1.0 / rate_hz)
        self.start()

    def wait_for_subscribers(self):
        # rclpy doesn't expose direct connection count; give a tiny delay
        self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))

    def update(self, x, y, z, th, speed, turn):
        with self.cond:
            self.x, self.y, self.z, self.th = x, y, z, th
            self.speed, self.turn = speed, turn
            self.cond.notify()

    def stop(self):
        self.done = True
        self.update(0,0,0,0,0,0)
        self.join()

    def run(self):
        while not self.done and rclpy.ok():
            with self.cond:
                notified = self.cond.wait(timeout=self.timeout)
                lx = self.x * self.speed
                ly = self.y * self.speed
                lz = self.z * self.speed
                az = self.th * self.turn

            if self.stamped:
                msg = TwistStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.twist.linear.x  = lx
                msg.twist.linear.y  = ly
                msg.twist.linear.z  = lz
                msg.twist.angular.z = az
            else:
                msg = Twist()
                msg.linear.x  = lx
                msg.linear.y  = ly
                msg.linear.z  = lz
                msg.angular.z = az
            self.pub.publish(msg)

        # Send zero on exit
        if self.stamped:
            zmsg = TwistStamped()
            zmsg.header.stamp = self.node.get_clock().now().to_msg()
            zmsg.header.frame_id = self.frame_id
            self.pub.publish(zmsg)
        else:
            self.pub.publish(Twist())

def main():
    settings = save_terminal()
    rclpy.init()
    node = Node('teleop_twist_keyboard')

    topic       = node.declare_parameter('topic', 'cmd_vel').get_parameter_value().string_value
    speed       = node.declare_parameter('speed', 0.25).get_parameter_value().double_value
    turn        = node.declare_parameter('turn',  0.5 ).get_parameter_value().double_value
    speed_limit = node.declare_parameter('speed_limit', 1.0).get_parameter_value().double_value
    turn_limit  = node.declare_parameter('turn_limit',  1.0).get_parameter_value().double_value
    repeat_rate = node.declare_parameter('repeat_rate', 20.0).get_parameter_value().double_value
    key_timeout = node.declare_parameter('key_timeout', 0.6).get_parameter_value().double_value
    stamped     = node.declare_parameter('stamped', False).get_parameter_value().bool_value
    frame_id    = node.declare_parameter('frame_id', '').get_parameter_value().string_value

    pub_thread = PublisherThread(node, topic, repeat_rate, stamped, frame_id)
    pub_thread.wait_for_subscribers()
    pub_thread.update(0,0,0,0, speed, turn)

    x = y = z = th = 0.0
    print(HELP)
    print(f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}")

    try:
        while rclpy.ok():
            key = getKey(key_timeout)

            if key in moveBindings:
                x, y, z, th = moveBindings[key]
            elif key in speedBindings:
                s_mul, t_mul = speedBindings[key]
                speed = min(speed_limit, speed * s_mul)
                turn  = min(turn_limit,  turn  * t_mul)
                print(f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}")
            else:
                if key == '':
                    pass  # keep last command if repeat_rate > 0
                else:
                    x = y = z = th = 0.0
                    if key == '\x03':
                        break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)
    finally:
        pub_thread.stop()
        restore_terminal(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
