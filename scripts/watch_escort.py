#!/usr/bin/env python3
"""Watch an escort run and shout when something crosses a safety line.

READ-ONLY BY CONSTRUCTION: this node creates subscriptions and nothing else.
No publishers, no service clients, no parameter writes. It cannot command a
drone, and it is safe to leave running across launches.

    python3 scripts/watch_escort.py                 # live, Ctrl-C for summary
    python3 scripts/watch_escort.py --log run1.jsonl

What it watches, and why each one is here:

* per-body mocap gaps on /poses, INCLUDING the DJI. A Crazyflie that loses
  tracking has no position measurement, its estimator drifts, and the show
  keeps commanding it from a model that cannot see it -- the 2026-10-07
  fly-away. Gaps are what precede that.
* each drone's radius from the arena centre, against the setpoint clamp
  (ARENA_RADIUS_PLAN), the containment trip, and the radius where tracking
  was actually lost.
* actual vs COMMANDED position. Divergence shows up here first, while the
  drone is still inside the room.
* min pairwise separation against min_pair_sep.
* ring altitude vs the DJI's. A defender at or below the DJI is the geometry
  the config calls unsurvivable, and a fence trip used to cause exactly that.
"""
import argparse, json, math, sys, time
from collections import deque

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data        # /poses is BEST_EFFORT
from crazyflie_interfaces.msg import FullState
from motion_capture_tracking_interfaces.msg import NamedPoseArray

from crazyflie_shows import escort, safety

YAML = 'src/crazyswarm2/crazyflie/config/crazyflies.yaml'
VIP_BODY = 'operator'


def enabled_fleet(path):
    d = yaml.safe_load(open(path))['robots']
    return sorted(k for k, v in d.items() if v.get('enabled'))


class Watch(Node):
    def __init__(self, fleet, cfg, logf):
        super().__init__('watch_escort')
        self.cfg, self.fleet, self.logf = cfg, fleet, logf
        self.pos, self.stamp = {}, {}
        self.cmd = {}
        self.alerts = deque(maxlen=400)
        self.worst = {'gap': 0.0, 'radius': 0.0, 'track': 0.0,
                      'sep': 9e9, 'ringgap': 9e9}
        self.create_subscription(NamedPoseArray, '/poses', self._poses,
                                 qos_profile_sensor_data)
        for n in fleet:
            self.create_subscription(
                FullState, f'/{n}/cmd_full_state',
                lambda m, n=n: self.cmd.__setitem__(
                    n, np.array([m.pose.position.x, m.pose.position.y,
                                 m.pose.position.z])), 10)
        self.t0 = time.monotonic()
        self.create_timer(0.25, self._tick)

    def _poses(self, msg):
        now = time.monotonic()
        for p in msg.poses:
            if not p.name:
                continue
            self.pos[p.name] = np.array([p.pose.position.x, p.pose.position.y,
                                         p.pose.position.z])
            self.stamp[p.name] = now

    def _say(self, level, text):
        t = time.monotonic() - self.t0
        line = f'  [{t:7.1f}s] {level}: {text}'
        print(line, flush=True)
        self.alerts.append(line)
        if self.logf:
            self.logf.write(json.dumps({'t': t, 'level': level,
                                        'msg': text}) + '\n')
            self.logf.flush()

    def _tick(self):
        now = time.monotonic()
        c, C = self.cfg, np.array(self.cfg.room_center)
        flying = [n for n in self.fleet if n in self.pos]
        if not flying and VIP_BODY not in self.pos:
            return

        # --- mocap gaps, every body including the DJI --------------------
        for name in list(self.stamp):
            gap = now - self.stamp[name]
            self.worst['gap'] = max(self.worst['gap'], gap)
            if gap > c.self_stale_land_s:
                self._say('LOST ', f'{name}: no mocap for {gap:.1f} s '
                                   f'(show lands at {c.self_stale_land_s:.1f})')
            elif gap > c.self_stale_s:
                self._say('stale', f'{name}: no mocap for {gap:.2f} s')

        trip = c.arena_radius + c.contain_margin
        for n in flying:
            p = self.pos[n]
            r = float(np.linalg.norm(p[:2] - C))
            self.worst['radius'] = max(self.worst['radius'], r)
            if r > safety.ARENA_RADIUS_LOST:
                self._say('LOST ', f'{n}: {r:.2f} m from centre -- past where '
                                   f'tracking was lost ({safety.ARENA_RADIUS_LOST:.2f})')
            elif r > trip:
                self._say('CONTA', f'{n}: {r:.2f} m from centre, past the '
                                   f'{trip:.2f} m containment')
            elif r > c.arena_radius:
                self._say('edge ', f'{n}: {r:.2f} m from centre, past the '
                                   f'{c.arena_radius:.2f} m clamp')
            sp = self.cmd.get(n)
            if sp is not None:
                err = float(np.linalg.norm(p[:2] - sp[:2]))
                self.worst['track'] = max(self.worst['track'], err)
                if err > c.track_error_m:
                    self._say('drift', f'{n}: {err:.2f} m from its command')

        for i, a in enumerate(flying):
            for b in flying[i + 1:]:
                d = float(np.linalg.norm(self.pos[a][:2] - self.pos[b][:2]))
                self.worst['sep'] = min(self.worst['sep'], d)
                if d < c.min_pair_sep:
                    self._say('SEP  ', f'{a}-{b} {d:.2f} m, under the '
                                       f'{c.min_pair_sep:.2f} m floor')

        # --- ring altitude vs the DJI ------------------------------------
        if VIP_BODY in self.pos and flying:
            vz = float(self.pos[VIP_BODY][2])
            if vz > 0.25:                      # only once the DJI is actually up
                for n in flying:
                    dz = float(self.pos[n][2]) - vz
                    self.worst['ringgap'] = min(self.worst['ringgap'], dz)
                    if dz < 0.05:
                        self._say('BELOW', f'{n} is {abs(dz):.2f} m BELOW the '
                                           'DJI -- downwash geometry')
                stray = float(np.linalg.norm(self.pos[VIP_BODY][:2] - C))
                if stray > escort.vip_keep_in(c):
                    self._say('fence', f'DJI {stray:.2f} m from centre, past '
                                       f'keep-in {escort.vip_keep_in(c):.2f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--log', default=None, help='append alerts as JSONL')
    a = ap.parse_args()
    cfg = escort.EscortConfig()
    fleet = enabled_fleet(YAML)
    logf = open(a.log, 'a') if a.log else None
    print(f'\n  WATCHING (read-only). fleet: {", ".join(fleet)}; VIP body {VIP_BODY!r}')
    print(f'  clamp {cfg.arena_radius:.2f} m | containment '
          f'{cfg.arena_radius + cfg.contain_margin:.2f} m | tracking lost '
          f'{safety.ARENA_RADIUS_LOST:.2f} m')
    print(f'  mocap hold {cfg.self_stale_s:.1f} s | land {cfg.self_stale_land_s:.1f} s'
          f' | pair floor {cfg.min_pair_sep:.2f} m\n')
    rclpy.init()
    w = Watch(fleet, cfg, logf)
    try:
        rclpy.spin(w)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass          # Ctrl-C or SIGTERM: still print the summary below
    print('\n  ==== worst seen ====')
    print(f'  longest mocap gap     {w.worst["gap"]:.2f} s')
    print(f'  furthest from centre  {w.worst["radius"]:.2f} m')
    print(f'  worst tracking error  {w.worst["track"]:.2f} m')
    s = w.worst['sep']
    print(f'  closest two drones    {"-" if s > 1e8 else f"{s:.2f} m"}')
    g = w.worst['ringgap']
    print(f'  least clearance over DJI {"-" if g > 1e8 else f"{g:+.2f} m"}')
    print(f'  alerts: {len(w.alerts)}')
    if logf:
        logf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
