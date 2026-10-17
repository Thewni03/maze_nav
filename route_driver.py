#!/usr/bin/env python3
"""Drive the TurtleBot3 along a fixed, slow tour of the maze so SLAM can map it.

v3: odometry for turns/distances + LiDAR wall-centring and heading correction while driving,
so wheel slip over a long tour cannot push the robot into the walls.
Publishes TwistStamped on /cmd_vel (what the Jazzy TurtleBot3 Gazebo bridge expects).

Usage:  python3 route_driver.py --full     # visit ALL 36 cells (final SLAM map, ~14 min)
        python3 route_driver.py            # short 11-leg perimeter tour
        python3 route_driver.py --legs 3   # only the first 3 legs
Start with the robot at its spawn pose (top-left cell, facing east). Ctrl+C stops it.
"""
import math
import sys

# (turn in degrees, +left / -right ; then drive in metres).  1 cell = 0.5 m
ROUTE = [
    (-90, 2.0), (90, 0.5), (-90, 0.5), (-90, 0.5), (180, 2.5), (90, 1.0),
    (180, 1.0), (-90, 2.0), (-90, 0.5), (90, 0.5), (-90, 2.0),
]
# Full-coverage tour: visits all 36 cells (depth-first, returns to the start). ~13 min.
ROUTE_FULL = [
    (-90, 2.0), (90, 0.5), (-90, 0.5), (90, 2.0),
    (90, 1.0), (180, 1.0), (-90, 2.5), (180, 0.5),
    (90, 0.5), (-90, 1.5), (90, 0.5), (90, 0.5),
    (-90, 0.5), (-90, 1.0), (90, 1.0), (90, 0.5),
    (90, 0.5), (-90, 0.5), (-90, 0.5), (90, 0.5),
    (90, 0.5), (-90, 0.5), (90, 1.0), (90, 0.5),
    (90, 0.5), (180, 0.5), (-90, 0.5), (-90, 1.5),
    (180, 0.5), (90, 0.5), (90, 0.5), (-90, 0.5),
    (-90, 0.5), (90, 0.5), (90, 0.5), (-90, 0.5),
    (-90, 1.0), (-90, 1.0), (90, 0.5), (90, 0.5),
    (-90, 0.5), (-90, 2.0), (-90, 2.0),
]
V = 0.08          # forward speed  m/s
W_TURN = 0.25     # max turn speed rad/s
W_HOLD = 0.30     # max heading-correction speed while driving
HALF = 0.225      # free half-width of a corridor (0.25 - wall half thickness)
K_H, K_LAT = 1.5, 3.0
WALL_NEAR = 0.40  # a side wall counts as present if closer than this
FRONT_STOP = 0.15


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def scan_features(ranges, angle_min, angle_inc):
    """Return dict(front, L, R, dl, dr). L/R = side distances, dl/dr = heading error (rad) of
    the robot relative to the left/right wall (None if that wall is not visible)."""
    n = len(ranges)

    def rng(deg):
        a = math.radians(deg) - angle_min
        i = int(round((a % (2 * math.pi)) / angle_inc)) % n
        vals = [ranges[(i + k) % n] for k in (-1, 0, 1)]
        vals = [v for v in vals if v is not None and math.isfinite(v) and v > 0.1]
        return min(vals) if vals else None

    def avg(degs):
        v = [rng(d) for d in degs]
        v = [x for x in v if x is not None]
        return sorted(v)[len(v) // 2] if v else None

    front = avg([-10, -5, 0, 5, 10])
    L, R = avg([85, 90, 95]), avg([-95, -90, -85])

    def heading(sign):                      # sign=+1 left wall, -1 right wall
        r1, r2 = rng(sign * 60), rng(sign * 120)
        if r1 is None or r2 is None or r1 > 0.6 or r2 > 0.6:
            return None
        p1 = (r1 * math.cos(math.radians(sign * 60)), r1 * math.sin(math.radians(sign * 60)))
        p2 = (r2 * math.cos(math.radians(sign * 120)), r2 * math.sin(math.radians(sign * 120)))
        d = -math.atan2(p1[1] - p2[1], p1[0] - p2[0])
        return d if abs(d) < math.radians(20) else None

    return dict(front=front, L=L, R=R, dl=heading(+1), dr=heading(-1))


class RouteController:
    """Pure logic (no ROS) so it can be tested. step() -> (v, w, done)."""

    def __init__(self, route, pause=1.0):
        self.route, self.pause = route, pause
        self.i, self.phase = 0, 'pause'
        self.t0 = self.yaw0 = self.ref = self.last_yaw = self.p0 = None
        self.turned = 0.0
        self.bias = 0.0


    @staticmethod
    def wall_heading(feat):
        """Heading error (rad) of the robot relative to the corridor walls, or None."""
        if not feat:
            return None
        L, R = feat['L'], feat['R']
        wl = L is not None and L < WALL_NEAR
        wr = R is not None and R < WALL_NEAR
        ds = [d for d, ok in ((feat['dl'], wl), (feat['dr'], wr)) if d is not None and ok]
        return sum(ds) / len(ds) if ds else None

    def step(self, x, y, yaw, t, feat=None):
        if self.yaw0 is None:
            self.yaw0, self.ref, self.t0 = yaw, yaw, t
        if self.i >= len(self.route):
            return 0.0, 0.0, True
        delta_deg, dist = self.route[self.i]
        if self.phase == 'pause':
            if t - self.t0 < self.pause:
                return 0.0, 0.0, False
            self.phase, self.turned, self.last_yaw = 'turn', 0.0, yaw
        if self.phase == 'turn':
            self.turned += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
            remaining = math.radians(delta_deg) - self.turned
            if abs(remaining) < math.radians(1.5):
                q = math.pi / 2
                self.ref = wrap(self.yaw0 + round(wrap(yaw - self.yaw0) / q) * q)  # snap to 90 deg grid
                self.phase, self.t0 = 'align', t
                return 0.0, 0.0, False
            w = min(W_TURN, max(0.08, 1.5 * abs(remaining)))
            return 0.0, math.copysign(w, remaining), False
        if self.phase == 'align':
            # use the corridor walls to remove the heading error left over from the turn
            d = self.wall_heading(feat)
            waited = t - self.t0
            if d is not None and abs(d) > math.radians(1.0) and waited < 4.0:
                w = max(0.05, min(0.15, 1.5 * abs(d)))
                return 0.0, -math.copysign(w, d), False
            if d is None and waited < 0.6:
                return 0.0, 0.0, False              # give the scan a moment
            self.bias = wrap(yaw - self.ref) - (d if d is not None else 0.0)
            self.phase, self.p0 = 'fwd', (x, y)
            return 0.0, 0.0, False
        # ---- forward ----
        left = dist - math.hypot(x - self.p0[0], y - self.p0[1])
        front = feat['front'] if feat else None
        if left <= 0.005 or (front is not None and front < FRONT_STOP and left < dist - 0.05):
            self.i += 1
            self.phase, self.t0 = 'pause', t
            return 0.0, 0.0, False
        v = min(V, max(0.03, 1.0 * left))
        if front is not None and front < 0.30:
            v = min(v, 0.04)
        offset, delta = 0.0, None
        if feat:
            L, R = feat['L'], feat['R']
            wl = L is not None and L < WALL_NEAR
            wr = R is not None and R < WALL_NEAR
            c = None                               # lateral displacement from corridor centre (left +)
            if wl and wr:
                c = (R - L) / 2
            elif wl:
                c = HALF - L
            elif wr:
                c = R - HALF
            if c is not None:
                offset = max(-0.2, min(0.2, -K_LAT * c))
            ds = [d for d, ok in ((feat['dl'], wl), (feat['dr'], wr)) if d is not None and ok]
            if ds:
                delta = sum(ds) / len(ds)
                new_bias = wrap(yaw - self.ref) - delta
                self.bias += 0.2 * wrap(new_bias - self.bias)
        if delta is not None:
            e = offset - delta
        else:
            e = wrap(self.ref + self.bias + offset - yaw)
        w = max(-W_HOLD, min(W_HOLD, K_H * e))
        return v, w, False


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    route = ROUTE_FULL if '--full' in sys.argv else ROUTE
    legs = len(route)
    if '--legs' in sys.argv:
        legs = int(sys.argv[sys.argv.index('--legs') + 1])

    class Driver(Node):
        def __init__(self):
            super().__init__('route_driver',
                             parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.ctrl = RouteController(route[:legs])
            self.pose, self.feat, self.last_leg = None, None, -1
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f'Waiting for /odom ... route has {legs} legs')

        def on_odom(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (p.x, p.y, yaw)

        def on_scan(self, m):
            self.feat = scan_features(list(m.ranges), m.angle_min, m.angle_increment)

        def send(self, v, w):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
            self.pub.publish(msg)

        def tick(self):
            if self.pose is None:
                return
            t = self.get_clock().now().nanoseconds * 1e-9
            v, w, done = self.ctrl.step(*self.pose, t, self.feat)
            self.send(v, w)
            if self.ctrl.i != self.last_leg and self.ctrl.i < legs:
                self.last_leg = self.ctrl.i
                self.get_logger().info(f'Leg {self.ctrl.i + 1}/{legs}: turn {route[self.ctrl.i][0]} deg, drive {route[self.ctrl.i][1]} m')
            if done:
                self.get_logger().info('Route complete.')
                raise SystemExit

    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        for _ in range(5):
            node.send(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
