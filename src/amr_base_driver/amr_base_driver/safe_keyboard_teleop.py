import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


HELP = '''
Safe keyboard control (press or hold a key)
-------------------------------------------
       W: forward
  A: left   D: right
       S: reverse

Space/X: STOP immediately
R: reset wheel odometry and dashboard distance while stopped
Q or Ctrl-C: STOP and quit

The command returns to zero automatically if no motion key is received.
Keep the physical motor cutoff within reach.
'''


class SafeKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('safe_keyboard_teleop')
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('key_timeout', 0.35)
        self.declare_parameter('publish_rate', 20.0)
        self.linear_speed = float(
            self.get_parameter('linear_speed').value)
        self.angular_speed = float(
            self.get_parameter('angular_speed').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.publish_rate = float(
            self.get_parameter('publish_rate').value)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_reset = self.create_client(Empty, '/reset_wheel_odometry')
        self.dashboard_reset = self.create_client(
            Empty, '/reset_dashboard_distance')
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key = 0.0
        self.last_status = None

    def set_key(self, key):
        key = key.lower()
        if key == 'w':
            self.linear, self.angular = self.linear_speed, 0.0
        elif key == 's':
            self.linear, self.angular = -self.linear_speed, 0.0
        elif key == 'a':
            self.linear, self.angular = 0.0, self.angular_speed
        elif key == 'd':
            self.linear, self.angular = 0.0, -self.angular_speed
        elif key in (' ', 'x'):
            self.stop('STOP')
            return True
        elif key == 'r':
            if self.linear or self.angular:
                self.stop('STOP before distance reset')
            if self.odom_reset.service_is_ready():
                self.odom_reset.call_async(Empty.Request())
            if self.dashboard_reset.service_is_ready():
                self.dashboard_reset.call_async(Empty.Request())
            self.show_status('DISTANCE RESET REQUESTED')
            return True
        else:
            return False
        self.last_motion_key = time.monotonic()
        return True

    def stop(self, reason='STOP'):
        self.linear = 0.0
        self.angular = 0.0
        self.last_motion_key = 0.0
        self.publish()
        self.show_status(reason)

    def expire_command(self):
        if (self.last_motion_key and
                time.monotonic() - self.last_motion_key > self.key_timeout):
            self.stop('AUTO STOP: release/keyboard timeout')

    def publish(self):
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def show_status(self, reason=None):
        status = (round(self.linear, 3), round(self.angular, 3), reason)
        if status == self.last_status:
            return
        self.last_status = status
        suffix = f' | {reason}' if reason else ''
        print(
            f'cmd_vel: linear={self.linear:+.3f} m/s  '
            f'angular={self.angular:+.3f} rad/s{suffix}',
            flush=True)


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('safe_keyboard_teleop requires an interactive TTY')
    rclpy.init(args=args)
    node = SafeKeyboardTeleop()
    previous_terminal = termios.tcgetattr(sys.stdin)
    period = 1.0 / max(node.publish_rate, 1.0)
    print(HELP)
    node.stop('READY; motors commanded to zero')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], period)
            if readable:
                key = sys.stdin.read(1)
                if key.lower() == 'q' or key == '\x03':
                    break
                if node.set_key(key):
                    node.show_status()
            node.expire_command()
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop('EXIT')
        for _ in range(4):
            node.publish()
            time.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous_terminal)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
