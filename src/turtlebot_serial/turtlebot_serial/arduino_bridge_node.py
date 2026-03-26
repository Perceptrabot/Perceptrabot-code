import math
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_pwm', 180)
        self.declare_parameter('linear_scale', 120.0)
        self.declare_parameter('turn_scale', 120.0)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('encoder_poll_hz', 20.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.turn_scale = float(self.get_parameter('turn_scale').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.encoder_poll_hz = float(self.get_parameter('encoder_poll_hz').value)

        self.serial = serial.Serial(
            port,
            baudrate=baud,
            timeout=0.0,
            write_timeout=0.1
        )

        time.sleep(2.0)  # allow Arduino reset after opening serial
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        self.last_cmd_time = time.monotonic()
        self.last_left = 0
        self.last_right = 0

        self.left_ticks = 0
        self.right_ticks = 0
        self.rx_buffer = ""

        # ---- Odometry calibration ----
        # Empirical calibration from your 1 m test
        self.meters_per_tick_left = 1.0 / 6022.0   # 0.00016606...
        self.meters_per_tick_right = 1.0 / 5871.0  # 0.00017033...
        self.wheel_base = 0.186  # meters (186 mm, center-to-center)

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_time = None

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.ticks_pub = self.create_publisher(
            Int64MultiArray,
            '/wheel_ticks',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.read_timer = self.create_timer(0.01, self.read_serial)  # 100 Hz
        self.encoder_timer = self.create_timer(
            1.0 / self.encoder_poll_hz,
            self.poll_encoders
        )

        # Zero encoder counts once at startup
        self.send_line("Z")

        self.get_logger().info('arduino_bridge ready')

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def send_line(self, line: str):
        try:
            self.serial.write((line + '\n').encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'serial write failed: {e}')

    def send_lr(self, left_pwm, right_pwm):
        if left_pwm == self.last_left and right_pwm == self.last_right:
            return

        self.send_line(f"LR {left_pwm} {right_pwm}")
        self.last_left = left_pwm
        self.last_right = right_pwm

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = time.monotonic()

        forward_pwm = int(-msg.linear.x * self.linear_scale)
        turn_pwm = int(msg.angular.z * self.turn_scale)

        left_pwm = forward_pwm - turn_pwm
        right_pwm = forward_pwm + turn_pwm

        left_pwm = self.clamp(left_pwm, -self.max_pwm, self.max_pwm)
        right_pwm = self.clamp(right_pwm, -self.max_pwm, self.max_pwm)

        self.send_lr(left_pwm, right_pwm)

    def watchdog_check(self):
        if time.monotonic() - self.last_cmd_time > self.timeout_sec:
            if self.last_left != 0 or self.last_right != 0:
                self.send_line("S")
                self.last_left = 0
                self.last_right = 0

    def poll_encoders(self):
        self.send_line("E?")

    def read_serial(self):
        try:
            waiting = self.serial.in_waiting
            if waiting <= 0:
                return

            data = self.serial.read(waiting)
            if not data:
                return

            self.rx_buffer += data.decode('ascii', errors='ignore')

            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                line = line.strip().strip('\r')
                if line:
                    self.handle_serial_line(line)

        except serial.SerialException as e:
            self.get_logger().error(f'serial read failed: {e}')

    def handle_serial_line(self, line: str):
        line = line.strip()
        if not line:
            return

        # Expected Arduino encoder format:
        # E,<left_raw>,<right_raw>
        if line.startswith("E,"):
            parts = line.split(",")
            if len(parts) == 3:
                try:
                    # Keep your current sign convention:
                    # raw left was opposite, raw right already correct
                    left = -int(parts[1])
                    right = int(parts[2])

                    self.left_ticks = left
                    self.right_ticks = right

                    ticks_msg = Int64MultiArray()
                    ticks_msg.data = [left, right]
                    self.ticks_pub.publish(ticks_msg)

                    self.update_odometry(left, right)
                    return

                except ValueError:
                    self.get_logger().warn(f'bad encoder line: {line}')
                    return

        # Ignore anything else quietly
        self.get_logger().debug(f'ignored serial line: {line}')

    def update_odometry(self, left_ticks, right_ticks):
        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        dL = delta_left_ticks * self.meters_per_tick_left
        dR = delta_right_ticks * self.meters_per_tick_right

        d_center = 0.5 * (dL + dR)
        d_theta = (dR - dL) / self.wheel_base

        self.x += d_center * math.cos(self.theta + 0.5 * d_theta)
        self.y += d_center * math.sin(self.theta + 0.5 * d_theta)
        self.theta = self.normalize_angle(self.theta + d_theta)

        v = d_center / dt
        w = d_theta / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        print(f"published odom {odom}")
        self.odom_pub.publish(odom)

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_time = now

    def destroy_node(self):
        try:
            self.send_line("S")
            self.serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()