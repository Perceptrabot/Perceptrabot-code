import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


MSG = """
Controls:
  w = forward
  s = backward
  a = turn left
  d = turn right
  space = stop
  q = quit
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # normalized command magnitudes
        self.linear_cmd = 0.8
        self.turn_cmd = 0.8

        print(MSG)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def publish_cmd(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.publisher.publish(msg)
        self.get_logger().info(f'Published cmd_vel: linear={lin}, angular={ang}')

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                self.publish_cmd(self.linear_cmd, 0.0)
            elif key == 's':
                self.publish_cmd(-self.linear_cmd, 0.0)
            elif key == 'a':
                self.publish_cmd(0.0,  self.turn_cmd)
            elif key == 'd':
                self.publish_cmd(0.0, -self.turn_cmd)
            elif key == ' ':
                self.publish_cmd(0.0, 0.0)
            elif key == 'q':
                self.publish_cmd(0.0, 0.0)
                break


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()