#!/usr/bin/env python3
"""Drive a virtual VIP, or the adversary, around the arena with the keyboard.

For testing the escort with nobody in the volume: instead of a person wearing
a mocap hat, a point you steer yourself.

    # terminal 2 -- the thing the defenders escort
    ros2 run crazyflie_shows escort_teleop --ros-args -p target:=vip

    # terminal 3 -- or the thing they block (a real drone flies to it)
    ros2 run crazyflie_shows escort_teleop --ros-args -p target:=adversary

    # terminal 1 -- the demo itself
    ros2 run crazyflie_shows escort_show --ros-args -p vip_mode:=manual

Keys, in PLAN VIEW -- the orientation ``plan_escort --plot`` draws, x to the
right and y up::

    w / up       +y            q  up   (adversary only; the VIP is on the floor)
    s / down     -y            e  down
    a / left     -x
    d / right    +x            space   stop where you are
                               c       re-centre on the start point
                               x       quit (the demo then sees a stale target)

Hold a key to keep moving: the terminal's auto-repeat keeps the velocity
alive, and it decays to zero ``key_timeout`` after the last keystroke. There
is no "release" event on a terminal, which is why it works this way rather
than as a held-key latch.

Two things this node will not let you do
----------------------------------------
* **Outrun the escort.** A VIP target defaults to ``escort.max_vip_speed`` --
  0.15 m/s with the shipped config -- because that is the speed budget the
  ring actually has left after turning. Steering a virtual person at 1 m/s
  proves nothing except that the defenders cannot keep up. Raise ``speed`` on
  purpose, knowing that is what you are testing.
* **Leave the room.** Every published point is clamped to the arena cylinder
  and, for the adversary, to the altitude band. The clamp is the same one the
  flight script applies, so what you steer is what it will try to fly.

Publishing stops when this node stops, and ``escort_show`` treats a stale
manual target exactly like a stale mocap pose: hold, then land. Quitting this
node is therefore a legitimate way to end a test, not a way to strand the
drones.

Parameters (``--ros-args -p name:=value``)
------------------------------------------
====================  ==========  ===================================
``target``            vip         ``vip`` | ``adversary``
``topic``             (derived)   ``/escort/vip`` or ``/escort/adversary``
``speed``             (derived)   m/s while a key is held
``step``              0.05        m per keystroke at low rates (see below)
``rate_hz``           20.0        publish rate
``key_timeout``       0.35        s of silence before the velocity decays
``x`` ``y`` ``z``     (derived)   start point; VIP defaults to the mark
====================  ==========  ===================================
"""

import select
import sys
import termios
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from crazyflie_shows import escort

VIP_TOPIC = '/escort/vip'
ADVERSARY_TOPIC = '/escort/adversary'

#: key -> unit step in the xy plane, plan view (x right, y up)
KEYS = {
    'w': (0.0, 1.0), 's': (0.0, -1.0), 'a': (-1.0, 0.0), 'd': (1.0, 0.0),
    '\x1b[A': (0.0, 1.0), '\x1b[B': (0.0, -1.0),
    '\x1b[D': (-1.0, 0.0), '\x1b[C': (1.0, 0.0),
}
Z_KEYS = {'q': 1.0, 'e': -1.0}


class Teleop(Node):

    def __init__(self):
        super().__init__('escort_teleop')
        self.cfg = escort.EscortConfig()
        self.target = self._p('target', 'vip')
        if self.target not in ('vip', 'adversary'):
            raise SystemExit("target must be 'vip' or 'adversary'")
        is_vip = self.target == 'vip'

        topic = self._p('topic', VIP_TOPIC if is_vip else ADVERSARY_TOPIC)
        # A VIP is a person on the floor; the adversary is a drone at the
        # defenders' altitude -- never above them, or its downwash lands on a
        # drone that cannot spare the thrust (ESCORT.md).
        home = escort.vip_home(self.cfg)
        if is_vip:
            start = np.array([self._p('x', float(home[0])),
                              self._p('y', float(home[1])), 0.0])
            default_speed = max(escort.max_vip_speed(self.cfg), 0.05)
        else:
            first = escort.AdversaryScript(height=self.cfg.height).target(0.0, home)
            start = np.array([self._p('x', float(first[0])),
                              self._p('y', float(first[1])),
                              self._p('z', float(self.cfg.height))])
            default_speed = 0.5

        self.speed = float(self._p('speed', default_speed))
        self.rate_hz = float(self._p('rate_hz', 20.0))
        self.key_timeout = float(self._p('key_timeout', 0.35))
        self.start = start.copy()
        self.pos = start.copy()
        self.vel = np.zeros(3)
        self.last_key = -1e9

        self.pub = self.create_publisher(PointStamped, topic, 10)
        print(f'\n  ESCORT TELEOP - driving the {self.target} on {topic}')
        print(f'  start  {np.round(self.pos, 2).tolist()}   speed {self.speed:.2f} m/s'
              f'{"  (= the ring speed budget; raise it on purpose)" if is_vip else ""}')
        print('  wasd / arrows move in plan view, q/e altitude, space stop, '
              'c centre, x quit\n')

    def _p(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def on_key(self, key, now):
        if key in KEYS:
            dx, dy = KEYS[key]
            self.vel = np.array([dx, dy, 0.0]) * self.speed
            self.last_key = now
        elif key in Z_KEYS and self.target == 'adversary':
            self.vel = np.array([0.0, 0.0, Z_KEYS[key] * self.speed])
            self.last_key = now
        elif key == ' ':
            self.vel = np.zeros(3)
        elif key == 'c':
            self.pos = self.start.copy()
            self.vel = np.zeros(3)
        elif key in ('x', '\x03'):
            return False
        return True

    def step(self, dt, now):
        if now - self.last_key > self.key_timeout:
            self.vel = np.zeros(3)
        p = self.pos + self.vel * dt

        # the room, clamped here so what you steer is what gets flown
        c = np.array(self.cfg.room_center)
        r = p[:2] - c
        n = float(np.linalg.norm(r))
        if n > self.cfg.arena_radius:
            p[:2] = c + r * (self.cfg.arena_radius / n)
        if self.target == 'adversary':
            p[2] = float(np.clip(p[2], self.cfg.floor, self.cfg.ceiling))
        else:
            p[2] = 0.0
        self.pos = p

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x, msg.point.y, msg.point.z = (float(p[0]), float(p[1]), float(p[2]))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Teleop()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    dt = 1.0 / node.rate_hz
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            now = node.get_clock().now().nanoseconds * 1e-9
            # Escape sequences arrive as 3 bytes; read what is waiting rather
            # than one byte, or an arrow key is seen as three keystrokes.
            if select.select([sys.stdin], [], [], dt)[0]:
                seq = sys.stdin.read(1)
                if seq == '\x1b':
                    while select.select([sys.stdin], [], [], 0.002)[0] and len(seq) < 3:
                        seq += sys.stdin.read(1)
                if not node.on_key(seq, now):
                    break
            node.step(dt, now)
            print(f'\r  {node.target} at '
                  f'[{node.pos[0]:+.2f}, {node.pos[1]:+.2f}, {node.pos[2]:.2f}]  '
                  f'|v| {np.linalg.norm(node.vel):.2f} m/s   ', end='', flush=True)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print('\n  teleop stopped - the demo will see a stale target and '
              'hold, then land.\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
