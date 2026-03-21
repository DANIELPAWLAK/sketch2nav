import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def grabKey(timeout: float = 0.05) -> str:
    """Non-blocking single key read from stdin."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ""


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")

        self.declare_parameter("linearSpeed", 0.6)
        self.declare_parameter("angularSpeed", 0.6)
        self.declare_parameter("publishRate", 20.0)

        self.linearSpeed = float(self.get_parameter("linearSpeed").value)
        self.angularSpeed = float(self.get_parameter("angularSpeed").value)
        rate_hz = float(self.get_parameter("publishRate").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            "Teleop:\n"
            "  W: forward\n"
            "  S: backward\n"
            "  A: pivot left\n"
            "  D: pivot right\n"
            "  Space: stop\n"
            "  Q: quit\n"
        )

    def on_timer(self):
        key = grabKey
    ()

        msg = Twist()

        if key in ("w", "W"):
            msg.linear.x = self.linearSpeed
            msg.angular.z = 0.0

        elif key in ("s", "S"):
            msg.linear.x = -self.linearSpeed
            msg.angular.z = 0.0

        elif key in ("a", "A"):
            msg.linear.x = 0.0
            msg.angular.z = self.angularSpeed

        elif key in ("d", "D"):
            msg.linear.x = 0.0
            msg.angular.z = -self.angularSpeed

        elif key == " ":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        elif key in ("q", "Q"):
            raise KeyboardInterrupt

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

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
