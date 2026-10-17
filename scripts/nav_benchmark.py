#!/usr/bin/env python3
"""Numbers for the report: Dijkstra vs A* (Nav2 NavFn planner) on three goals.

Run with Nav2 active and the robot at its start pose (top-left cell):
    python3 ~/nav_benchmark.py            # phase 1 only: planning time + path length (robot does not move)
    python3 ~/nav_benchmark.py --drive    # also drives each goal; stops at the first failure
Results are printed and saved to ~/nav_planning.csv and ~/nav_driving.csv
"""
import csv
import math
import os
import subprocess
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

# goal name: (x, y) in the map frame (start cell centre = (0,0); 1 cell = 0.5 m; x east, y north), optimal grid path (m)
GOALS = {
    'G1 bottom-left (r5,c0)': ((0.0, -2.5), 3.5),
    'G2 bottom-right (r5,c5)': ((2.5, -2.5), 5.0),
    'G3 top-right (r0,c5)': ((2.5, 0.0), 4.5),
}
PLANNERS = {'Dijkstra': 'false', 'A*': 'true'}
TIMEOUT = 150.0   # sim seconds allowed per drive
REPEATS = 10


def path_len(poses):
    return sum(math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)
               for a, b in zip(poses, poses[1:]))


class Bench(Node):
    def __init__(self):
        super().__init__('nav_benchmark',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.last = None
        self.dist = 0.0
        self.track = False

    def on_odom(self, m):
        p = m.pose.pose.position
        if self.track and self.last is not None:
            self.dist += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.last = (p.x, p.y)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self, x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.orientation.w = float(x), float(y), 1.0
        return p

    def at_start(self, tol=0.35):
        return self.last is not None and math.hypot(self.last[0], self.last[1]) < tol

    def set_planner(self, flag):
        r = subprocess.run(['ros2', 'param', 'set', '/planner_server', 'GridBased.use_astar', flag],
                           capture_output=True, text=True)
        if 'successful' not in r.stdout:
            print('WARNING: could not set planner:', r.stdout.strip(), r.stderr.strip())

    def plan_only(self, x, y):
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose(x, y)
        goal.planner_id = 'GridBased'
        goal.use_start = False
        f = self.plan.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted:
            return None, None
        r = gh.get_result_async()
        rclpy.spin_until_future_complete(self, r)
        res = r.result().result
        if len(res.path.poses) < 2:
            return None, None
        return (res.planning_time.sec + res.planning_time.nanosec * 1e-9) * 1000.0, path_len(res.path.poses)

    def drive(self, x, y, track=True):
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y)
        self.dist, self.track = 0.0, track
        f = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted:
            self.track = False
            return False, 0.0
        t0 = self.now()
        r = gh.get_result_async()
        while rclpy.ok() and not r.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.now() - t0 > TIMEOUT:
                gh.cancel_goal_async()
                break
        t = self.now() - t0
        self.track = False
        return (r.done() and r.result().status == GoalStatus.STATUS_SUCCEEDED), t


def save(name, head, rows):
    with open(os.path.expanduser(name), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(head)
        w.writerows(rows)
    print('Saved', os.path.expanduser(name))


def main():
    drive = '--drive' in sys.argv
    rclpy.init()
    b = Bench()
    print('Waiting for Nav2 ...')
    b.nav.wait_for_server()
    b.plan.wait_for_server()
    while b.last is None:
        rclpy.spin_once(b, timeout_sec=0.2)
    if not b.at_start():
        print('ERROR: the robot is not at its start pose. Restart the simulation and Nav2, then run again.')
        return

    # ---- phase 1: planning only (robot stays at the start) ----
    head = ['goal', 'planner', 'ok_runs', 'plan_ms_avg', 'plan_len_m', 'optimal_grid_m', 'excess_pct']
    rows = []
    for gname, ((gx, gy), opt) in GOALS.items():
        for pname, flag in PLANNERS.items():
            b.set_planner(flag)
            ts, ln = [], None
            for _ in range(REPEATS):
                t, ln_i = b.plan_only(gx, gy)
                if t is not None:
                    ts.append(t)
                    ln = ln_i
            if ts:
                rows.append([gname, pname, f'{len(ts)}/{REPEATS}', round(sum(ts) / len(ts), 2), round(ln, 2), opt,
                             round(100 * (ln - opt) / opt, 1)])
            else:
                rows.append([gname, pname, f'0/{REPEATS}', 'no path', '-', opt, '-'])
    print('\nPLANNING (robot at start)\n' + ' | '.join(head))
    for r in rows:
        print(' | '.join(str(c) for c in r))
    save('~/nav_planning.csv', head, rows)

    if not drive:
        print('\nPhase 1 done. Add --drive to also drive the goals.')
        return

    # ---- phase 2: driving ----
    head = ['goal', 'planner', 'reached', 'travel_s', 'driven_m', 'straight_m', 'efficiency(straight/driven)', 'avg_speed_mps']
    rows = []
    for gname, ((gx, gy), opt) in GOALS.items():
        straight = math.hypot(gx, gy)
        for pname, flag in PLANNERS.items():
            if not b.at_start():
                print('Robot is not at the start - stopping so the results stay valid.')
                break
            b.set_planner(flag)
            ok, t = b.drive(gx, gy)
            d = b.dist
            eff = round(straight / d, 2) if ok and d > 0.1 else 'n/a'
            rows.append([gname, pname, 'yes' if ok else 'NO', round(t, 1), round(d, 2), round(straight, 2), eff,
                         round(d / t, 3) if t > 0 else 'n/a'])
            print('  ', rows[-1])
            if not ok:
                print('Drive failed - stopping. Look at the Nav2 terminal for the reason.')
                break
            for _ in range(2):                              # return to the start
                b.drive(0.0, 0.0, track=False)
                if b.at_start():
                    break
            if not b.at_start():
                print('Could not return to the start - stopping.')
                break
        else:
            continue
        break
    print('\nDRIVING\n' + ' | '.join(head))
    for r in rows:
        print(' | '.join(str(c) for c in r))
    save('~/nav_driving.csv', head, rows)
    b.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
