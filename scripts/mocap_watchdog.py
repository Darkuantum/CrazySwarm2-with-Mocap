#!/usr/bin/env python3
"""Mocap watchdog for crazyswarm2 — prevents fly-away when mocap drops.

The Crazyflie cannot tell that mocap stopped: if /poses stalls, the EKF keeps
integrating IMU, the position estimate drifts, and the controller chases it into
a wall. This node watches /poses freshness and, if it stalls for longer than
--timeout while flying, cuts the motors (or lands) on ALL drones.

Run alongside the stack (needs the crazyflie stack + pose_bridge.py up):

    python3 mocap_watchdog.py                       # cut motors after 150 ms gap
    python3 mocap_watchdog.py --timeout 0.25        # more tolerant
    python3 mocap_watchdog.py --action land         # controlled land instead (uses drifting estimate)

Emergency is the only action that *guarantees* no fly-away. Land is gentler but
relies on the same estimate that's already going bad, so it can still drift.
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from std_srvs.srv import Empty
from crazyflie_interfaces.srv import Land


class MocapWatchdog(Node):
    def __init__(self, timeout, action, land_height, land_duration):
        super().__init__('mocap_watchdog')
        self.timeout = timeout
        self.action = action
        self.land_height = land_height
        self.land_duration = land_duration
        self.last = None          # time of last /poses message
        self.seen = False         # have we ever received poses (i.e. mocap was up)
        self.triggered = False

        # match pose_bridge.py's publisher QoS (BEST_EFFORT) or we get nothing
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(NamedPoseArray, '/poses', self.on_poses, qos)

        if action == 'emergency':
            self.cli = self.create_client(Empty, '/all/emergency')
        else:
            self.cli = self.create_client(Land, '/all/land')

        self.create_timer(0.02, self.check)   # 50 Hz check
        self.get_logger().info(
            f'Mocap watchdog armed: action={action.upper()}, '
            f'timeout={timeout * 1000:.0f} ms, watching /poses')

    def on_poses(self, _msg):
        self.last = self.get_clock().now()
        self.seen = True
        if self.triggered:
            self.triggered = False
            self.get_logger().info('Mocap resumed — watchdog re-armed')

    def check(self):
        if not self.seen or self.triggered or self.last is None:
            return
        gap = (self.get_clock().now() - self.last).nanoseconds * 1e-9
        if gap > self.timeout:
            self.triggered = True
            self.get_logger().error(
                f'MOCAP LOST ({gap * 1000:.0f} ms gap) -> firing {self.action.upper()}')
            self.fire()

    def fire(self):
        if not self.cli.service_is_ready():
            self.cli.wait_for_service(timeout_sec=0.5)
        if self.action == 'emergency':
            self.cli.call_async(Empty.Request())
        else:
            req = Land.Request()
            req.group_mask = 0
            req.height = self.land_height
            req.duration = Duration(seconds=self.land_duration).to_msg()
            self.cli.call_async(req)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--timeout', type=float, default=0.15,
                   help='max /poses gap in seconds before firing (default 0.15)')
    p.add_argument('--action', choices=['emergency', 'land'], default='emergency',
                   help='emergency = cut motors (default); land = controlled descent')
    p.add_argument('--land-height', type=float, default=0.05, help='land action target height (m)')
    p.add_argument('--land-duration', type=float, default=2.0, help='land action duration (s)')
    args = p.parse_args()

    rclpy.init()
    node = MocapWatchdog(args.timeout, args.action, args.land_height, args.land_duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
