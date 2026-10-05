#!/usr/bin/env python3
"""Show the escort's marks in RViz and tell you how far each drone is from one.

For the job of walking four drones onto their marks and chalking the floor.
``plan_escort --marks`` prints the coordinates, which is no help when you are
crouched on the floor holding a Crazyflie: this draws them where they are, in
the same world frame as the live mocap, and counts the error down to zero.

    # mocap up, server NOT needed (this touches no radio)
    python3 scripts/place_drones.py

In RViz add a MarkerArray on ``/escort/marks`` (config.rviz already has the
Escort display on /escort/markers -- this is a SECOND topic, so add one).

What you see per mark
---------------------
* a flat disc on the floor at the mark, with the role and the drone currently
  nearest it;
* a vertical post, because a disc seen from across the room is a line;
* once a drone is close, a line from the drone to its mark, which shrinks as
  you slide it in -- when the line vanishes you are inside the tolerance.

The marks come from ``plan_escort.recommend_marks``, the same function that
prints them and that ``check_config`` validates, so what you stand on is what
the show was planned against. Edit that config while this is running and the
marks move by themselves -- it watches the mtime of ``escort.py`` and
``plan_escort.py`` and reloads, because a mark that is one config revision out
of date is not a stale display, it is a wrong chalk line on the floor.

Nothing here writes the yaml: once the drones are down, run
``scripts/sync_initial_positions.py`` so the yaml records where they ACTUALLY
are rather than where they were meant to go.
"""

import argparse
import importlib
import math
import os
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from crazyflie_shows import escort, plan_escort
from motion_capture_tracking_interfaces.msg import NamedPoseArray

TOPIC = '/escort/marks'
FRAME = 'world'
C_MARK = (0.30, 0.75, 1.00, 0.55)     # an empty mark
C_CLOSE = (1.00, 0.70, 0.10, 0.85)    # a drone is near it, not yet on it
C_ON = (0.20, 0.95, 0.35, 0.90)       # within tolerance -- chalk here
C_VIP = (1.00, 1.00, 1.00, 0.70)
C_LINK = (1.00, 0.45, 0.10, 0.85)


def _mtimes():
    """When the geometry's source files last changed."""
    return tuple(os.path.getmtime(m.__file__) for m in (escort, plan_escort))


def build_marks():
    """(labelled marks, VIP point) from the CURRENT config.

    Recomputed rather than captured at startup. The marks are chalked on the
    floor from this picture, so a config edit that is not reflected here is
    not a stale display -- it is a wrong chalk mark, and it happened once
    (2026-10-01: ring_radius changed under a running instance and the RViz
    discs kept showing the old ring until it was restarted).
    """
    cfg = escort.EscortConfig()
    d_marks, a_mark = plan_escort.recommend_marks(cfg)
    marks = [(f'defender {i + 1}', np.asarray(m, float)[:2])
             for i, m in enumerate(d_marks)]
    marks.append(('ADVERSARY', np.asarray(a_mark, float)[:2]))
    return marks, escort.vip_home(cfg)


def reload_marks():
    """Re-import the geometry and rebuild. Returns (marks, vip)."""
    importlib.reload(escort)
    importlib.reload(plan_escort)
    return build_marks()


def _marker(mid, kind, scale, colour, ns='marks'):
    m = Marker()
    m.header.frame_id = FRAME
    m.ns, m.id, m.type, m.action = ns, int(mid), kind, Marker.ADD
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)  # ROS wants floats
    m.color = ColorRGBA(r=float(colour[0]), g=float(colour[1]),
                        b=float(colour[2]), a=float(colour[3]))
    m.pose.orientation.w = 1.0
    m.lifetime = Duration(sec=2, nanosec=0)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--tol', type=float, default=0.10,
                    help='m. Inside this the mark turns green -- chalk it.')
    ap.add_argument('--rate', type=float, default=5.0)
    args = ap.parse_args()

    marks, vip = build_marks()
    stamp = _mtimes()

    rclpy.init()
    node = Node('place_drones')
    pub = node.create_publisher(MarkerArray, TOPIC, 1)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    live = {}
    node.create_subscription(
        NamedPoseArray, '/poses',
        lambda msg: live.update({p.name: (p.pose.position.x, p.pose.position.y)
                                 for p in msg.poses}), qos)

    def announce(why):
        print(f'\n  {why} -- publishing {TOPIC} (add a MarkerArray on it)')
        print(f'  green at {args.tol * 100:.0f} cm. '
              'Ctrl-C when the floor is marked.\n')
        for name, m in marks:
            print(f'    {name:<11} [{m[0]:+.2f}, {m[1]:+.2f}]')
        print(f'    {"VIP (none)":<11} [{vip[0]:+.2f}, {vip[1]:+.2f}]  '
              'nothing stands here -- it is the point the ring holds\n')

    announce('PLACE THE DRONES')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
            now_stamp = _mtimes()
            if now_stamp != stamp:                # the geometry changed under us
                stamp = now_stamp
                try:
                    marks, vip = reload_marks()
                    announce('CONFIG CHANGED - these marks are new')
                except Exception as exc:          # noqa: BLE001  mid-edit file
                    print(f'\n  config will not import yet ({exc}) -- '
                          'still showing the last good marks\n')
            arr = MarkerArray()
            report = []
            for k, (name, m) in enumerate(marks):
                # whichever drone is nearest this mark owns it
                near, d = None, 1e9
                for drone, p in live.items():
                    dd = math.dist(p, m)
                    if dd < d:
                        near, d = drone, dd
                colour = (C_ON if d <= args.tol else
                          C_CLOSE if d <= 0.6 else C_MARK)
                disc = _marker(k * 10, Marker.CYLINDER, (0.34, 0.34, 0.012), colour)
                disc.pose.position.x, disc.pose.position.y = float(m[0]), float(m[1])
                disc.pose.position.z = 0.006
                arr.markers.append(disc)
                post = _marker(k * 10 + 1, Marker.CYLINDER, (0.02, 0.02, 0.9), colour)
                post.pose.position.x, post.pose.position.y = float(m[0]), float(m[1])
                post.pose.position.z = 0.45
                arr.markers.append(post)
                txt = _marker(k * 10 + 2, Marker.TEXT_VIEW_FACING, (0, 0, 0.17), colour)
                txt.pose.position.x, txt.pose.position.y = float(m[0]), float(m[1])
                txt.pose.position.z = 1.0
                txt.text = (f'{name}  {near or "-"}  {d * 100:.0f}cm'
                            if near and d < 1.5 else name)
                arr.markers.append(txt)
                if near and args.tol < d < 1.5:
                    link = _marker(k * 10 + 3, Marker.LINE_LIST, (0.015, 0, 0), C_LINK)
                    link.points = [
                        Point(x=float(live[near][0]), y=float(live[near][1]), z=0.05),
                        Point(x=float(m[0]), y=float(m[1]), z=0.05)]
                    arr.markers.append(link)
                report.append(f'{name.split()[0][:3]}:{near or "-"} '
                              f'{d * 100:5.0f}cm{"  ON" if d <= args.tol else ""}')

            v = _marker(90, Marker.SPHERE, (0.2, 0.2, 0.2), C_VIP)
            v.pose.position.x, v.pose.position.y, v.pose.position.z = (
                float(vip[0]), float(vip[1]), 0.1)
            arr.markers.append(v)
            vt = _marker(91, Marker.TEXT_VIEW_FACING, (0, 0, 0.16), C_VIP)
            vt.pose.position.x, vt.pose.position.y, vt.pose.position.z = (
                float(vip[0]), float(vip[1]), 0.35)
            vt.text = 'VIP point (keep clear)'
            arr.markers.append(vt)

            pub.publish(arr)
            print('   ' + ' | '.join(report) + '        ', end='\r', flush=True)
    except KeyboardInterrupt:
        print('\n\n  marks published until now. Once the drones are down, run:')
        print('    python3 scripts/sync_initial_positions.py\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
