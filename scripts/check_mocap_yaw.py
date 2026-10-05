#!/usr/bin/env python3
"""Live yaw of every mocap rigid body, for fixing rigid-body orientation.

Run this while you square the drones and reset their orientation in Motive.
It prints, a few times a second, where each body's OWN X axis points in the
world -- which is what the flight stack believes the drone's nose is doing.

    # mocap up, server NOT running (this touches no radio)
    python3 scripts/check_mocap_yaw.py

Why this matters more than it looks
-----------------------------------
`crazyflie_server.cpp` sends the full pose, quaternion included, whenever the
quaternion is valid (`sendExternalPoses`); only a NaN quaternion falls back to
position-only. With `tracking: "vendor"` it is always valid, so **mocap yaw is
fused into each drone's estimator**. A rigid body whose X axis does not lie
along the airframe's nose therefore tells the drone it is facing somewhere it
is not, and its position corrections push the wrong way: destabilisation and
drift within a second or two of takeoff. At rest it looks perfect, which is
what makes it dangerous (CLAUDE.md, "Rigid-body orientation").

How to use it
-------------
1. Point every drone's nose along world +X. In Motive that is the axis the
   ground-plane triad calls X -- check the triad, not your memory of where the
   calibration square went.
2. Watch this. Every drone should read yaw about 0 deg.
3. Any drone that does not: its rigid body was defined while it sat at that
   angle. In Motive, select that body and reset its orientation (so the
   current pose becomes identity), or delete and recreate it with the drone
   squared. Then watch this again.
4. Only fly when every drone reads within a few degrees of 0.

A drone reading 180 deg is the nastiest case: it looks symmetric, flies
backwards into its own corrections, and no amount of position accuracy saves
it.
"""

import argparse
import sys

import numpy as np

TOL_GOOD = 5.0     # deg -- CLAUDE.md: within 5 is flyable
TOL_FIX = 15.0     # deg -- 5..15 fix first; beyond 15, do not fly


def yaw_deg(q):
    x, y, z, w = q
    return float(np.degrees(np.arctan2(2 * (w * z + x * y),
                                       1 - 2 * (y * y + z * z))))


def rp_deg(q):
    x, y, z, w = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return float(roll), float(pitch)


def verdict(yaw):
    a = abs(yaw)
    if a <= TOL_GOOD:
        return 'OK'
    if a <= TOL_FIX:
        return 'FIX FIRST'
    if a > 150.0:
        return 'BACKWARDS?'
    return 'DO NOT FLY'


def nose_test(body, secs, topic, rclpy, Node, qos, NamedPoseArray):
    """Push the drone along its own nose; compare travel with reported yaw.

    The one check a freshly created rigid body cannot fake. Motive gives a new
    body identity orientation, so it reports yaw 0 whatever the nose does --
    "yaw is 0" says nothing about whether the body frame follows the airframe.
    But if the body sits a degrees off the nose, pushing the drone straight
    forward travels a degrees away from the bearing it reports. That gap is
    the offset, and it is what flew cf1 into the floor.
    """
    import numpy as np
    pos, yaws = [], []

    class Rec(Node):
        def __init__(self):
            super().__init__('nose_test')
            self.create_subscription(NamedPoseArray, topic, self.cb, qos)

        def cb(self, msg):
            for e in msg.poses:
                if e.name == body:
                    p, o = e.pose.position, e.pose.orientation
                    pos.append([p.x, p.y, p.z])
                    yaws.append(yaw_deg((o.x, o.y, o.z, o.w)))

    rclpy.init()
    node = Rec()
    print(f'\n  recording {body} for {secs:.0f} s -- PUSH IT FORWARD ALONG ITS '
          'NOSE now, a metre or more\n')
    end = node.get_clock().now().nanoseconds * 1e-9 + secs
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if pos:
                d = float(np.linalg.norm(np.array(pos[-1])[:2] - np.array(pos[0])[:2]))
                left = end - node.get_clock().now().nanoseconds * 1e-9
                print(f'\r  moved {d:.2f} m, {left:.0f} s left   ', end='', flush=True)
    except KeyboardInterrupt:
        pass
    print()
    P, Y = np.array(pos), np.array(yaws)
    if len(P) < 20:
        print(f'  only {len(P)} samples -- is {body} tracked, and is the mocap '
              'node running?\n')
        rclpy.shutdown()
        return 1
    travel = P[-1] - P[0]
    dist = float(np.linalg.norm(travel[:2]))
    if dist < 0.3:
        print(f'  {body} moved {dist:.2f} m -- under 30 cm is not enough to '
              'measure a bearing. Push it further and run this again.\n')
        rclpy.shutdown()
        return 1
    bearing = float(np.degrees(np.arctan2(travel[1], travel[0])))
    yaw = float(np.degrees(np.arctan2(np.mean(np.sin(np.radians(Y))),
                                      np.mean(np.cos(np.radians(Y))))))
    off = (bearing - yaw + 180) % 360 - 180
    print(f'  moved {dist:.2f} m along {bearing:+.1f} deg')
    print(f'  body reported yaw {yaw:+.1f} deg (spread {Y.max() - Y.min():.1f})')
    print(f'\n  ==> body-vs-nose offset {off:+.1f} deg')
    if abs(off) <= 5:
        print('      GOOD -- the body frame follows the airframe.\n')
    elif abs(off) <= 15:
        print('      FIX FIRST -- 5-15 deg is the fix-before-flying band.\n')
    elif abs(off) > 150:
        print('      BACKWARDS -- ~180 deg out. Do not arm.\n')
    else:
        print('      DO NOT FLY -- this is the offset that crashed cf1.\n')
    rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--topic', default='/poses')
    ap.add_argument('--hz', type=float, default=4.0)
    ap.add_argument('--once', action='store_true', help='one reading, then exit')
    ap.add_argument('--nose', metavar='BODY',
                    help='push BODY along its own nose while this records, to '
                         'prove the body frame follows the airframe')
    ap.add_argument('--seconds', type=float, default=45.0,
                    help='how long --nose records for')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from motion_capture_tracking_interfaces.msg import NamedPoseArray

    if args.nose:
        return nose_test(args.nose, args.seconds, args.topic,
                         rclpy, Node, qos_profile_sensor_data, NamedPoseArray)

    latest = {}

    class Watch(Node):
        def __init__(self):
            super().__init__('check_mocap_yaw')
            # /poses is published with sensor QoS (best effort). A default
            # reliable subscription silently receives NOTHING from it.
            self.create_subscription(NamedPoseArray, args.topic, self.cb,
                                     qos_profile_sensor_data)

        def cb(self, msg):
            for e in msg.poses:
                o, p = e.pose.orientation, e.pose.position
                latest[e.name] = ((o.x, o.y, o.z, o.w), (p.x, p.y, p.z))

    rclpy.init()
    node = Watch()
    print(__doc__.split('How to use it')[0].strip()[:0] or '', end='')
    print('\n  point every nose along world +X; each should read yaw ~ 0\n')
    print(f'  {"body":<8}{"yaw":>8}{"roll":>8}{"pitch":>8}   {"x":>7}{"y":>7}'
          f'{"z":>7}   verdict')
    try:
        while rclpy.ok():
            for _ in range(int(max(1.0, 50.0 / max(args.hz, 0.1)))):
                rclpy.spin_once(node, timeout_sec=0.02)
            if not latest:
                print('\r  waiting for /poses ...', end='', flush=True)
                continue
            lines = []
            for name in sorted(latest):
                q, p = latest[name]
                y = yaw_deg(q)
                r, pi = rp_deg(q)
                lines.append(f'  {name:<8}{y:+8.1f}{r:+8.1f}{pi:+8.1f}   '
                             f'{p[0]:+7.2f}{p[1]:+7.2f}{p[2]:+7.2f}   {verdict(y)}')
            print('\n'.join(lines))
            if args.once:
                break
            print(f'\033[{len(lines)}A', end='')      # redraw in place
    except KeyboardInterrupt:
        print('\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
