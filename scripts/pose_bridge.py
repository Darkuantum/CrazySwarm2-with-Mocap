#!/usr/bin/env python3
"""natnet_ros2 -> /poses bridge: the ALTERNATIVE mocap path, not the default.

The default path is the vendored ``motion_capture_tracking`` node started by
``launch.py``, which talks to Motive directly and publishes ``/poses`` itself.
This script exists for the case where you want the open natnet_ros2 driver
instead: it collects each rigid body's ``PoseStamped`` and republishes them as
the single ``NamedPoseArray`` on ``/poses`` that ``crazyflie_server`` consumes.

Three things were wrong with it until 2026-10-06, all of them silent:

* **The roster was hardcoded** to ``['cf1', 'cf2']`` and had been wrong for
  months. It is now read from ``crazyflies.yaml``, which is the only roster in
  this workspace.
* **It subscribed to ``/<name>/pose``, which ``crazyflie_server`` also
  publishes** -- that topic carries the drone's *own* onboard estimate. With
  both publishers live, this bridge republished a mix of mocap truth and the
  drone's own EKF output back into ``/poses``, which the server force-fuses as
  ground truth at ``locSrv.extPosStdDev = 1e-3``. It now reads the namespaced
  ``/mocap/<name>/pose`` (``natnet_ros2.launch.py`` defaults to
  ``namespace:=mocap``), so the two can no longer collide.
* **Nothing ever expired a pose.** ``latest`` only grew, so a rigid body that
  Motive stopped tracking was republished frozen at its last position for as
  long as the bridge ran -- a dropout presented to the server as a drone
  holding perfectly still. Entries now go stale and are dropped.

Run it with the stack up but *no* mocap node of its own::

    ros2 launch natnet_ros2 natnet_ros2.launch.py     # publishes /mocap/<body>/pose
    python3 pose_bridge.py
"""

import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

#: Hz. Must match what Motive streams (50 Hz on this rig) -- publishing faster
#: than the data arrives just repeats samples, and the server's own pose age
#: checks then never see a dropout. docs/MOCAP.md quoted this constant for a
#: while before it existed; the file hardcoded 240 in two places.
PUBLISH_HZ = 50.0

#: A pose older than this is dropped rather than republished. Three missed
#: frames at 50 Hz: long enough not to flicker, short enough that the server
#: sees a real dropout as a dropout.
STALE_AFTER_S = 0.25

#: Topic namespace natnet_ros2 publishes under. Keep in step with the
#: ``namespace`` launch argument (default ``mocap``). Empty string = the old
#: colliding behaviour.
DEFAULT_NAMESPACE = 'mocap'

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'src', 'crazyswarm2', 'crazyflie', 'config', 'crazyflies.yaml')


def enabled_drones(yaml_path=DEFAULT_YAML):
    """Names of the enabled drones, from the one roster in this workspace."""
    with open(yaml_path) as fh:
        robots = (yaml.safe_load(fh) or {}).get('robots') or {}
    return [name for name, r in robots.items() if (r or {}).get('enabled')]


class PoseBridge(Node):

    def __init__(self):
        super().__init__('pose_bridge')
        self.declare_parameter('yaml', DEFAULT_YAML)
        self.declare_parameter('namespace', DEFAULT_NAMESPACE)
        self.declare_parameter('publish_hz', PUBLISH_HZ)
        self.declare_parameter('stale_after_s', STALE_AFTER_S)

        yaml_path = self.get_parameter('yaml').value
        ns = self.get_parameter('namespace').value.strip('/')
        hz = float(self.get_parameter('publish_hz').value)
        self.stale_after = float(self.get_parameter('stale_after_s').value)

        drones = enabled_drones(yaml_path)
        if not drones:
            raise SystemExit(f'no enabled drones in {yaml_path}')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos.deadline = Duration(seconds=0, nanoseconds=int(1e9 / hz))
        self.poses_pub = self.create_publisher(NamedPoseArray, '/poses', qos)

        self.latest = {}        # name -> (PoseStamped, rx monotonic seconds)
        self._warned_stale = set()
        prefix = f'/{ns}' if ns else ''
        for name in drones:
            topic = f'{prefix}/{name}/pose'
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, n=name: self.callback(msg, n), 10)

        self.create_timer(1.0 / hz, self.publish_all)
        if not ns:
            self.get_logger().warning(
                'namespace is empty, so this bridge subscribes to /<name>/pose '
                '-- the SAME topic crazyflie_server publishes the onboard '
                'estimate on. If the server is running, /poses will carry the '
                "drones' own estimates back to the server as mocap truth.")
        self.get_logger().info(
            f'pose bridge started for {drones} on {prefix}/<name>/pose '
            f'at {hz:g} Hz (from {yaml_path})')

    def callback(self, msg, name):
        self.latest[name] = (msg, self._now())

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_all(self):
        now = self._now()
        arr = NamedPoseArray()
        header = None
        for name, (msg, rx) in sorted(self.latest.items()):
            if now - rx > self.stale_after:
                # Do NOT republish it: a frozen pose is indistinguishable from
                # a drone holding still, and the server force-fuses it.
                if name not in self._warned_stale:
                    self._warned_stale.add(name)
                    self.get_logger().warning(
                        f'{name}: no pose for {now - rx:.2f} s - dropping it '
                        'from /poses until Motive tracks it again')
                continue
            self._warned_stale.discard(name)
            header = msg.header
            arr.poses.append(NamedPose(name=name, pose=msg.pose))
        if not arr.poses:
            return
        arr.header = header
        self.poses_pub.publish(arr)


def main():
    rclpy.init()
    try:
        rclpy.spin(PoseBridge())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
