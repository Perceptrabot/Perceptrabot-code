import threading

from geometry_msgs.msg import Twist
from pynput import keyboard
import rclpy
from rclpy.node import Node


MSG = """
Smooth teleop:
  hold w/s : ramp linear forward/backward
  hold a/d : ramp angular left/right
  space    : stop immediately
  q/esc    : quit

This version tracks key press/release state, so linear and angular
are updated independently.
"""


class SmoothTeleop(Node):
    def __init__(self):
        super().__init__('smooth_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.linear_cmd = 0.0
        self.angular_cmd = 0.0

        self.linear_accel = 0.10
        self.angular_accel = 0.12
        self.linear_max = 1.0
        self.angular_max = 1.0
        self.update_period = 0.05

        self.pressed = {'w': False, 's': False, 'a': False, 'd': False}
        self.lock = threading.Lock()
        self.should_quit = False

        self.timer = self.create_timer(self.update_period, self.update_and_publish)
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )

        print(MSG)
        self.print_state()

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def move_toward(self, current, target, step):
        if current < target:
            return min(target, current + step)
        if current > target:
            return max(target, current - step)
        return current

    def print_state(self):
        print(
            f"\rlinear={self.linear_cmd:+.2f}  angular={self.angular_cmd:+.2f}      ",
            end="",
            flush=True,
        )

    def _char_from_key(self, key):
        if isinstance(key, keyboard.KeyCode):
            return key.char
        return None

    def on_press(self, key):
        char = self._char_from_key(key)

        with self.lock:
            if key == keyboard.Key.space:
                self.linear_cmd = 0.0
                self.angular_cmd = 0.0
            elif key == keyboard.Key.esc or char == 'q':
                self.should_quit = True
            elif char in self.pressed:
                self.pressed[char] = True

    def on_release(self, key):
        char = self._char_from_key(key)

        with self.lock:
            if char in self.pressed:
                self.pressed[char] = False

    def update_and_publish(self):
        with self.lock:
            linear_target = 0.0
            angular_target = 0.0

            if self.pressed['w'] and not self.pressed['s']:
                linear_target = self.linear_max
            elif self.pressed['s'] and not self.pressed['w']:
                linear_target = -self.linear_max

            if self.pressed['a'] and not self.pressed['d']:
                angular_target = self.angular_max
            elif self.pressed['d'] and not self.pressed['a']:
                angular_target = -self.angular_max

            self.linear_cmd = self.clamp(
                self.move_toward(self.linear_cmd, linear_target, self.linear_accel),
                -self.linear_max,
                self.linear_max,
            )
            self.angular_cmd = self.clamp(
                self.move_toward(self.angular_cmd, angular_target, self.angular_accel),
                -self.angular_max,
                self.angular_max,
            )

            msg = Twist()
            msg.linear.x = round(self.linear_cmd, 2)
            msg.angular.z = round(self.angular_cmd, 2)
            self.pub.publish(msg)

            self.print_state()

    def run(self):
        self.listener.start()
        try:
            while rclpy.ok() and not self.should_quit:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            with self.lock:
                self.linear_cmd = 0.0
                self.angular_cmd = 0.0

            self.pub.publish(Twist())
            self.listener.stop()
            print("\nQuitting.")


def main(args=None):
    rclpy.init(args=args)
    node = SmoothTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
