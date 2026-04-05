#!/usr/bin/env python3
"""
sketch2nav_control — Pure Pursuit path follower
================================================
Drive type : skid-steer / 4-wheel independent (treated as differential drive)
Subscribes : /sketch_path  (nav_msgs/Path)   — from the web UI
             /odom          (nav_msgs/Odometry) — from the Gazebo bridge
Publishes  : /cmd_vel       (geometry_msgs/Twist)
             /sketch_path_markers (visualization_msgs/MarkerArray) — RViz debug

ROS2 parameters (tune via config/params.yaml or CLI):
  lookahead_distance  float  0.5   Pure pursuit lookahead radius (m)
  max_linear_speed    float  0.3   Forward speed cap (m/s)
  min_linear_speed    float  0.05  Minimum speed so the robot keeps moving
  goal_tolerance      float  0.15  Distance at which goal is declared reached (m)
  angular_gain        float  1.5   Scales angular velocity output
  max_angular_speed   float  2.0   Angular velocity cap (rad/s)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PurePursuit(Node):

    def __init__(self):
        super().__init__('purePursuit')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_linear_speed',   0.3)
        self.declare_parameter('min_linear_speed',   0.05)
        self.declare_parameter('goal_tolerance',     0.15)
        self.declare_parameter('angular_gain',       1.5)
        self.declare_parameter('max_angular_speed',  2.0)

        # ── Robot state ───────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # ── Path state ────────────────────────────────────────
        self.path:     list[tuple[float, float]] = []
        self.path_idx: int  = 0   # lookahead search start — advances forward only
        self.active:   bool = False

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(Path,     '/sketch_path', self._path_cb, 10)
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE    
            )       
        self.create_subscription(Odometry, '/odom', self._odom_cb, odom_qos)

        # ── Publishers ────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',              10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sketch_path_markers',  10)

        # ── 20 Hz control loop ────────────────────────────────
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Pure pursuit ready — waiting for /sketch_path')

    # ──────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn('Empty path received — ignoring.')
            return

        self.path     = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_idx = 0
        self.active   = True
        self.get_logger().info(f'New path: {len(self.path)} waypoints')
        self._publish_markers()

    def _odom_cb(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Quaternion → yaw (rotation about Z)
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # ──────────────────────────────────────────────────────────
    # Control loop
    # ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        if not self.active or not self.path:
            return
            
        # TEMP: log every 20 ticks (once per second) to confirm loop is running
        if not hasattr(self, '_tick'): self._tick = 0
        self._tick += 1
        if self._tick % 20 == 0:
            self.get_logger().info(f'pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}°  idx={self.path_idx}/{len(self.path)}')

        ld      = self.get_parameter('lookahead_distance').value
        max_v   = self.get_parameter('max_linear_speed').value
        min_v   = self.get_parameter('min_linear_speed').value
        tol     = self.get_parameter('goal_tolerance').value
        k_ang   = self.get_parameter('angular_gain').value
        max_w   = self.get_parameter('max_angular_speed').value

        # ── Goal check ────────────────────────────────────────
        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) < tol:
            self._stop()
            self.active = False
            self.get_logger().info('Goal reached!')
            return

        # ── Find lookahead target ─────────────────────────────
        target = self._find_lookahead(ld)
        if target is None:
            # All path points are behind us — drive straight to goal
            target = self.path[-1]

        tx, ty = target
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return

        # ── Pure pursuit geometry ─────────────────────────────
        # alpha = signed angle between robot heading and vector to target
        alpha = self._norm_angle(math.atan2(dy, dx) - self.yaw)

        # Curvature: κ = 2·sin(α) / L  (L = lookahead distance)
        curvature = 2.0 * math.sin(alpha) / max(dist, 0.01)

        # Scale forward speed down proportionally with heading error
        # — full speed when straight, min_v when 90°+
        speed_scale = max(0.0, 1.0 - abs(alpha) / math.pi)
        v = max(min_v, max_v * speed_scale)

        # Skid-steer: angular velocity from curvature
        omega = k_ang * curvature * v
        omega = max(-max_w, min(max_w, omega))

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────
    # Lookahead search
    # ──────────────────────────────────────────────────────────

    def _find_lookahead(self, ld: float) -> tuple[float, float] | None:
        robot = (self.x, self.y)

        # Only advance path_idx if we're genuinely close to that point
        while self.path_idx < len(self.path) - 1:
            dist_to_current = math.hypot(
                self.path[self.path_idx][0] - robot[0],
                self.path[self.path_idx][1] - robot[1]
            )   
            if dist_to_current > ld * 0.5:
                break
            self.path_idx += 1

        # Search from path_idx forward for a circle intersection
        for i in range(self.path_idx, len(self.path) - 1):
            hit = self._circle_segment_intersect(robot, ld, self.path[i], self.path[i + 1])
            if hit is not None:
                return hit

    # No intersection found — return the next unvisited waypoint directly
    # so the robot always has something to drive toward
        if self.path_idx < len(self.path):
            return self.path[self.path_idx]

        return None
    
    def _circle_segment_intersect(
        self,
        center: tuple[float, float],
        r: float,
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> tuple[float, float] | None:
        """
        Intersect the circle (center, r) with segment a→b.
        Returns the intersection point with the highest parameter t (closest
        to b, i.e. furthest along the path), or None if no intersection.
        """
        cx, cy = center
        ax, ay = a[0] - cx, a[1] - cy
        bx, by = b[0] - cx, b[1] - cy

        dx, dy = bx - ax, by - ay
        dr2    = dx * dx + dy * dy
        if dr2 < 1e-10:
            return None

        D    = ax * by - ay * bx
        disc = r * r * dr2 - D * D
        if disc < 0:
            return None

        sqrt_disc = math.sqrt(disc)
        sign_dy   = 1.0 if dy >= 0 else -1.0

        candidates: list[tuple[float, float, float]] = []
        for xi, yi in [
            ( D * dy + sign_dy * dx * sqrt_disc,
             -D * dx + abs(dy)  * sqrt_disc),
            ( D * dy - sign_dy * dx * sqrt_disc,
             -D * dx - abs(dy)  * sqrt_disc),
        ]:
            xi /= dr2
            yi /= dr2
            # Parametric position along segment
            t = (xi - ax) / dx if abs(dx) > abs(dy) else (yi - ay) / dy
            if 0.0 <= t <= 1.0:
                candidates.append((t, xi + cx, yi + cy))

        if not candidates:
            return None

        # Pick point furthest along the segment
        candidates.sort(key=lambda c: c[0], reverse=True)
        return (candidates[0][1], candidates[0][2])

    # ──────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _norm_angle(a: float) -> float:
        """Wrap angle to [-π, π]."""
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ──────────────────────────────────────────────────────────
    # RViz markers
    # ──────────────────────────────────────────────────────────

    def _publish_markers(self) -> None:
        """Publish the path as a line strip + waypoint spheres for RViz."""
        if not self.path:
            return

        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Line strip
        line               = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp    = stamp
        line.ns              = 'sketch_path'
        line.id              = 0
        line.type            = Marker.LINE_STRIP
        line.action          = Marker.ADD
        line.scale.x         = 0.03
        line.color           = ColorRGBA(r=0.0, g=0.9, b=0.63, a=0.8)
        line.pose.orientation.w = 1.0
        for wx, wy in self.path:
            p = Point(); p.x = wx; p.y = wy; p.z = 0.02
            line.points.append(p)
        array.markers.append(line)

        # Waypoint spheres (subsampled to avoid flooding RViz)
        step = max(1, len(self.path) // 20)
        for i, (wx, wy) in enumerate(self.path[::step]):
            m = Marker()
            m.header.frame_id    = 'odom'
            m.header.stamp       = stamp
            m.ns                 = 'sketch_waypoints'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = wx
            m.pose.position.y    = wy
            m.pose.position.z    = 0.04
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color = ColorRGBA(r=0.0, g=0.9, b=0.63, a=0.6)
            array.markers.append(m)

        self.marker_pub.publish(array)


# ──────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
