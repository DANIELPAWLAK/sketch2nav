#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class PurePursuit(Node):
    def __init__(self):
        super().__init__('purePursuit')
        self.declare_parameter('lookaheadDistance', 0.5)
        self.declare_parameter('maxLinearSpeed',    0.3)
        self.declare_parameter('minLinearSpeed',    0.05)
        self.declare_parameter('goalTolerance',     0.15)
        self.declare_parameter('angularGain',       1.5)
        self.declare_parameter('maxAngularSpeed',   2.0)
        self.robotX   = 0.0
        self.robotY   = 0.0
        self.robotYaw = 0.0
        self.path    = []
        self.pathIdx = 0
        self.active  = False

        self.create_subscription(Path, '/sketch_path', self.pathCallback, 10)
        self.create_subscription(Bool, '/sketch_stop', self.stopCallback, 10)
        odomQos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Odometry, '/odom', self.odomCallback, odomQos)
        self.cmdPub    = self.create_publisher(Twist,       '/cmd_vel',             10)
        self.markerPub = self.create_publisher(MarkerArray, '/sketch_path_markers', 10)
        self.create_timer(0.05, self.controlLoop)
        self.get_logger().info('Pure pursuit ready — waiting for /sketch_path')

    def pathCallback(self, msg):
        if not msg.poses:
            self.get_logger().warn('Empty path received — ignoring.')
            return
        self.path    = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.pathIdx = 0
        self.active  = True
        self.get_logger().info(f'New path: {len(self.path)} waypoints')
        self.publishMarkers()

    def odomCallback(self, msg):
        self.robotX = msg.pose.pose.position.x
        self.robotY = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robotYaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def stopCallback(self, msg):
        if msg.data:
            self.active = False
            self.stop()
            self.get_logger().info('Stop received.')

    def controlLoop(self):
        if not self.active or not self.path:
            return
        lookahead = self.get_parameter('lookaheadDistance').value
        maxV      = self.get_parameter('maxLinearSpeed').value
        minV      = self.get_parameter('minLinearSpeed').value
        tol       = self.get_parameter('goalTolerance').value
        kAng      = self.get_parameter('angularGain').value
        maxW      = self.get_parameter('maxAngularSpeed').value
        
        gx, gy = self.path[-1]
        if math.hypot(gx - self.robotX, gy - self.robotY) < tol:
            self.stop()
            self.active = False
            self.get_logger().info('Goal reached!')
            return
            
        target = self.findLookahead(lookahead)
        if target is None:
            # All path points are behind us — drive straight to goal
            target = self.path[-1]

        tx, ty = target
        dx   = tx - self.robotX
        dy   = ty - self.robotY
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return
        alpha = self.normAngle(math.atan2(dy, dx) - self.robotYaw)
        curvature = 2.0 * math.sin(alpha) / max(dist, 0.01)
        speedScale = max(0.0, 1.0 - abs(alpha) / math.pi)
        v = max(minV, maxV * speedScale)
        omega = kAng * curvature * v
        omega = max(-maxW, min(maxW, omega))
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        self.cmdPub.publish(cmd)


    def findLookahead(self, lookahead):
        robotPos = (self.robotX, self.robotY)
        while self.pathIdx < len(self.path) - 1:
            distToCurrent = math.hypot(
                self.path[self.pathIdx][0] - robotPos[0],
                self.path[self.pathIdx][1] - robotPos[1]
            )
            if distToCurrent > lookahead * 0.5:
                break
            self.pathIdx += 1
        for i in range(self.pathIdx, len(self.path) - 1):
            hit = self.circleSegmentIntersect(robotPos, lookahead, self.path[i], self.path[i + 1])
            if hit is not None:
                return hit
        if self.pathIdx < len(self.path):
            return self.path[self.pathIdx]

        return None

    def circleSegmentIntersect(self, center, r, a, b):
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
        sqrtDisc = math.sqrt(disc)
        signDy   = 1.0 if dy >= 0 else -1.0
        candidates = []
        for xi, yi in [
            ( D * dy + signDy * dx * sqrtDisc,
             -D * dx + abs(dy) * sqrtDisc),
            ( D * dy - signDy * dx * sqrtDisc,
             -D * dx - abs(dy) * sqrtDisc),
        ]:
            xi /= dr2
            yi /= dr2
            # Parametric position along segment
            t = (xi - ax) / dx if abs(dx) > abs(dy) else (yi - ay) / dy
            if 0.0 <= t <= 1.0:
                candidates.append((t, xi + cx, yi + cy))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0], reverse=True)
        return (candidates[0][1], candidates[0][2])


    def stop(self):
        self.cmdPub.publish(Twist())

    def normAngle(self, a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def publishMarkers(self):
        if not self.path:
            return
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id    = 'odom'
        line.header.stamp       = stamp
        line.ns                 = 'sketch_path'
        line.id                 = 0
        line.type               = Marker.LINE_STRIP
        line.action             = Marker.ADD
        line.scale.x            = 0.03
        line.color              = ColorRGBA(r=0.0, g=0.9, b=0.63, a=0.8)
        line.pose.orientation.w = 1.0
        for wx, wy in self.path:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.02
            line.points.append(p)
        array.markers.append(line)
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
        self.markerPub.publish(array)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
