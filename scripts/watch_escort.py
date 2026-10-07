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
#: Only call a drone "below the DJI" when it is within this horizontal
#: distance of it. Outside that the altitude difference is irrelevant.
DOWNWASH_RXY = 0.80
#: Must match escort_show.FENCE_HYST -- this file re-derives the fence
#: timeline independently, so a different hysteresis would report trips the
#: show did not take.
FENCE_HYST = 0.25


def enabled_fleet(path):
    d = yaml.safe_load(open(path))['robots']
    return sorted(k for k, v in d.items() if v.get('enabled'))


class Watch(Node):
    def __init__(self, fleet, cfg, logf):
        super().__init__('watch_escort')
        self.cfg, self.fleet, self.logf = cfg, fleet, logf
        self.pos, self.stamp = {}, {}
        self.cmd = {}
        self.cmd_t = {}          # when each command last arrived
        self.alerts = deque(maxlen=400)
        self.worst = {'gap': 0.0, 'radius': 0.0, 'track': 0.0,
                      'sep': 9e9, 'ringgap': 9e9}
        self.create_subscription(NamedPoseArray, '/poses', self._poses,
                                 qos_profile_sensor_data)
        for n in fleet:
            self.create_subscription(
                FullState, f'/{n}/cmd_full_state',
                lambda m, n=n: (self.cmd.__setitem__(
                    n, np.array([m.pose.position.x, m.pose.position.y,
                                 m.pose.position.z])),
                    self.cmd_t.__setitem__(n, time.monotonic())), 10)
        self.t0 = time.monotonic()
        self.streaming = None        # is the show commanding anyone?
        self.fence = {'on': True, 'since': None, 'wall': None,
                      'centre': None, 'last_good': None, 'holds': []}
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
        # Wall clock on EVERY line, not just elapsed. The point of this
        # monitor is to be laid against a video recording, and after the
        # 2026-10-07 run the LED changes could not be matched to geofence
        # events because nothing printed an absolute time.
        t = time.monotonic() - self.t0
        wall = time.time()
        line = f'  [{t:7.1f}s {time.strftime("%H:%M:%S", time.localtime(wall))}] {level}: {text}'
        print(line, flush=True)
        self.alerts.append(line)
        if self.logf:
            self.logf.write(json.dumps(
                {'t': round(t, 3), 'wall': wall,
                 'clock': time.strftime('%Y-%m-%dT%H:%M:%S',
                                        time.localtime(wall)),
                 'level': level, 'msg': text}) + '\n')
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

        # Is the show actually flying? Without this a dropout cannot be told
        # apart from "nothing was running", which is exactly the question the
        # 2026-10-07 DJI dropout left open: a 10.2 s VIP loss that should have
        # landed the fleet, with no way to know whether the show was up.
        live_cmd = sorted(n for n in self.fleet
                          if now - self.cmd_t.get(n, 0.0) <= 0.5)
        if bool(live_cmd) != bool(self.streaming):
            self.streaming = live_cmd
            self._say('show ', (f'STREAMING to {", ".join(live_cmd)}'
                                if live_cmd else 'stopped streaming'))
        self.streaming = live_cmd

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
            # Only against a LIVE command. The show stops streaming when it
            # lands, and comparing a landed drone against the last setpoint it
            # ever got produces a large CONSTANT error that looks exactly like
            # a drone failing to follow orders. Measured 2026-10-07: a 1.21 m
            # "tracking error" that ran for 180 s straight through a stack
            # shutdown, and which I reported as a real flight problem.
            sp = self.cmd.get(n)
            if sp is not None and now - self.cmd_t.get(n, 0.0) > 0.5:
                sp = None
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
                    # Horizontal distance matters: a drone 6 cm lower than the
                    # DJI but 1.8 m away from it is in nobody's downwash, and
                    # flagging that produced a 28 s "BELOW" episode for a run
                    # that was visibly fine. Only complain when it is actually
                    # underneath.
                    dxy = float(np.linalg.norm(self.pos[n][:2]
                                               - self.pos[VIP_BODY][:2]))
                    dz = float(self.pos[n][2]) - vz
                    if dxy > DOWNWASH_RXY:
                        continue
                    self.worst['ringgap'] = min(self.worst['ringgap'], dz)
                    if dz < 0.05:
                        self._say('BELOW', f'{n} is {abs(dz):.2f} m below the '
                                           f'DJI and {dxy:.2f} m from it '
                                           '-- downwash geometry')
                self._fence(now, C)

    def _fence(self, now, C):
        """An INDEPENDENT fence timeline, computed from mocap alone.

        The show prints its own GEOFENCE lines, but those are the show's
        claim about itself; this is the same predicate evaluated from /poses
        by a process that commands nothing, so the two can be diffed. It
        exists because of a question a live run could not answer
        (2026-10-07): the defenders changed colour several times and nobody
        could say which of those were fence trips. Transitions only, with
        durations -- the previous version re-reported the same stray four
        times a second for as long as the DJI was out.

        Mirrors escort_show's predicate deliberately, including its
        asymmetry: it trips on stray > keep_in, and clears only once the DJI
        is back inside keep_in - FENCE_HYST *and* within ring_radius of the
        point the ring parked on. Drift here from escort_show is a bug in
        this file; keep them together.
        """
        c, f = self.cfg, self.fence
        keep_in = escort.vip_keep_in(c)
        p = self.pos[VIP_BODY]
        stray = float(np.linalg.norm(p[:2] - C))
        if f['on']:
            if stray > keep_in:
                f['on'] = False
                f['since'] = now
                # The ring parks on the last position it was still following,
                # not on where the DJI has got to.
                f['centre'] = (f['last_good'] if f['last_good'] is not None
                               else p).copy()
                f['wall'] = time.time()
                self._say('FENCE', f'HOLD -- DJI {stray:.2f} m from centre, '
                                   f'past keep-in {keep_in:.2f}; ring should '
                                   'stop following and hold altitude')
            else:
                f['last_good'] = p.copy()
        else:
            in_arena = stray <= keep_in - FENCE_HYST
            in_ring = float(np.linalg.norm(p[:2] - f['centre'][:2])) <= c.ring_radius
            if in_arena and in_ring:
                held = now - f['since']
                f['on'] = True
                f['last_good'] = p.copy()
                f['holds'].append((f['wall'], held))
                self._say('FENCE', f'RESUME after {held:.1f} s -- DJI back '
                                   f'{stray:.2f} m from centre and inside the '
                                   'parked ring')


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
    h = w.fence['holds']
    if not w.fence['on'] and w.fence.get('wall'):
        h = h + [(w.fence['wall'], time.time() - w.fence['since'])]
    if h:
        print(f'  fence holds: {len(h)}')
        for i, (wall, held) in enumerate(h, 1):
            clock = time.strftime('%H:%M:%S', time.localtime(wall))
            tail = ('  (still held at exit)'
                    if not w.fence['on'] and i == len(h) else '')
            print(f'    {i:<3} {clock}   held {held:5.1f} s{tail}')
    else:
        print('  fence holds: none -- the DJI stayed inside keep-in')
    if logf:
        logf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
