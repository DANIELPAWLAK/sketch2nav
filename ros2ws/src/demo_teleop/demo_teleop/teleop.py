import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key(timeout: float = 0.05) -> str:
    """Non-blocking single key read from stdin."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ""


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")

        self.declare_parameter("linear_step", 0.15)
        self.declare_parameter("angular_step", 0.6)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.v = 0.0
        self.w = 0.0

        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            "Teleop:\n"
            "  W/S: forward/back\n"
            "  A/D: left/right (yaw)\n"
            "  Space: stop\n"
            "  Q: quit\n"
        )

    def on_timer(self):
        key = get_key()

        if key in ("w", "W"):
            self.v += self.linear_step
        elif key in ("s", "S"):
            self.v -= self.linear_step
        elif key in ("a", "A"):
            self.w += self.angular_step
        elif key in ("d", "D"):
            self.w -= self.angular_step
        elif key == " ":
            self.v = 0.0
            self.w = 0.0
        elif key in ("q", "Q"):
            raise KeyboardInterrupt

        # Clamp (keeps it controllable)
        self.v = max(min(self.v, 1.0), -1.0)
        self.w = max(min(self.w, 3.0), -3.0)

        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)
        self.pub.publish(msg)


def main():
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.init()
        node = Teleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
