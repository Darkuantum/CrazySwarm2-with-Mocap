#!/usr/bin/env python3
"""Record the "link alive, log data dead" stall with its own precursor evidence.

Why this exists
---------------
On 2026-10-02 one or more drones repeatedly stopped delivering ALL firmware log
data -- /cfX/pose, /cfX/status and /cfX/kalman_preflight all at exactly 0.00 Hz,
not degraded -- while the same drone kept answering latency pings (~0.84/s at
10-35 ms) and kept accepting parameter writes. A second Crazyradio proved the
drone itself was healthy: a fresh connection streamed its log data at 10 Hz
immediately, with 2.2 h of drone uptime, so nothing on the aircraft had crashed.

Six hours of that session went into reconstructing each event from
`~/.ros/log/<run>/launch.log` AFTER the fact, which misled us at least once: a
"when did it die" timeline built from the *last warning* per drone was an
artifact, because those warnings only fire while a rate is BAD -- a healthy
drone emits none. This script exists so the next occurrence arrives already
measured.

What it records
---------------
Continuously, once a second, per enabled drone:
  * pose rate over a trailing window (the symptom),
  * what the DRONE says it received, from /cfX/status (num_rx_unicast and
    num_rx_broadcast). These are the decisive numbers now: the firmware deletes
    every log block by itself once the drone has received NOTHING for 1000 ms
    (log.c logRunBlock -> crtpIsConnected -> radiolink.c, same dispatch that
    increments these counters), so a dip here in the second before a freeze is
    the trigger, directly observed. A healthy drone reads ~170 and ~145.
  * the server-side link counters from /cfX/connection_statistics
    (needs `warnings.communication.publish_stats: true` in server.yaml):
    sent, sent_ping, receive, enqueued, ack.
and, on every STALL (a drone that was delivering drops to zero and stays there),
dumps a JSON file holding the preceding PRECURSOR_S seconds for EVERY drone --
not just the victim -- because the open question is whether the others were
disturbed at the same instant.

The counters are what make the dump decisive:
  sent flat                      -> the radio loop stopped polling that drone
  sent rising, ack flat          -> the drone stopped acking (an RF problem)
  ack rising, receive flat       -> acks arrive, payloads discarded
                                    (safelink down-bit mismatch, or ack filter)
  receive rising, enqueued flat  -> received but never queued to the application

It also tags every row with how the stack was launched -- the parent process of
`ros2 launch` -- because whether console-launched stacks stall more often than
shell-launched ones is still unsettled, and one run each proves nothing.

Usage
-----
    # with the stack already running (any backend, any launcher)
    python3 scripts/link_stall_recorder.py                  # Ctrl-C to stop
    python3 scripts/link_stall_recorder.py --out data/linkstalls --window 5

Nothing here commands a drone: it only subscribes. Safe to leave running during
flights, though it is most useful on the ground where stalls were observed.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
import time
from collections import defaultdict, deque

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

WORKSPACE = pathlib.Path(__file__).resolve().parent.parent
FLEET_YAML = WORKSPACE / 'src/crazyswarm2/crazyflie/config/crazyflies.yaml'

#: How much history every stall dump carries, per drone.
PRECURSOR_S = 60.0
#: A drone counts as stalled once this long has passed with NO pose at all.
#:
#: This used to be ZERO_SECONDS = 4 consecutive seconds of a zero *windowed
#: rate*, which with a 5 s window needed ~9 s of silence to fire. That was
#: fine while only a server restart could recover a stall, and became unable
#: to fire at all once the telemetry watchdog landed (server.yaml
#: telemetry_watchdog_s: 5.0): the watchdog declares at 5 s and has the log
#: blocks back ~1 s later, so the pose stream resumes before the rate ever
#: reads 0.00 four times running, zero_run tops out at 1-2 and dump() never
#: runs. eadcda1 edited this file AFTER the watchdog landed, saying the
#: recorder would now "show the outage itself", while leaving the trigger
#: unable to see it. Absence of stall dumps then reads as absence of stalls --
#: exactly the trap CLAUDE.md records being burned by on 2026-10-02.
#:
#: A gap is also the right measurement: it is what the drone's own firmware
#: times out on (1 s without receiving anything -> logReset + crtpReset).
STALL_GAP_S = 2.0
#: ...but only if it had been delivering at least this much beforehand.
ALIVE_HZ = 3.0
#: Wait this long after declaring a stall before writing the dump, so that the
#: watchdog's recovery -- and the server's "log blocks RECREATED" line -- land
#: INSIDE the captured window instead of just after it.
DUMP_DELAY_S = 6.0


def enabled_drones(path=FLEET_YAML):
    doc = yaml.safe_load(open(path))
    return [n for n, c in (doc.get('robots') or {}).items()
            if isinstance(c, dict) and c.get('enabled')]


def launcher():
    """Who started the stack: 'console', 'shell', or 'unknown'.

    The console-vs-terminal question is still open, and a label on every row is
    the only way to settle it with more than one sample per side.
    """
    try:
        out = subprocess.run(['ps', '-eo', 'pid,ppid,cmd'], capture_output=True,
                             text=True, timeout=5).stdout
    except Exception:                                        # noqa: BLE001
        return 'unknown'
    launch_ppid = None
    for line in out.splitlines():
        if 'ros2 launch crazyflie' in line and 'grep' not in line:
            parts = line.split(None, 2)
            if len(parts) >= 2:
                launch_ppid = parts[1]
            break
    if launch_ppid is None:
        return 'no-launch-process'
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) >= 3 and parts[0] == launch_ppid:
            cmd = parts[2]
            if 'mission_console' in cmd:
                return 'console'
            if cmd.split('/')[-1].split()[0] in ('bash', 'zsh', 'sh', 'fish'):
                return 'shell'
            return f'other:{cmd.split()[0][:40]}'
    return 'unknown'


class Recorder(Node):
    def __init__(self, names, out_dir, window):
        super().__init__('link_stall_recorder')
        self.names, self.window = names, window
        self.out_dir = pathlib.Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.launcher = launcher()
        self.stamps = {n: deque(maxlen=4000) for n in names}       # pose arrivals
        self.stats = {n: None for n in names}                      # latest counters
        self.rx = {n: None for n in names}                         # drone-side rx
        self.history = {n: deque(maxlen=int(PRECURSOR_S) + 10) for n in names}
        self.last_pose = {n: None for n in names}   # last arrival, per drone
        self.was_alive = defaultdict(bool)
        self.stalled = set()
        self.pending_dump = {}                      # name -> when to dump
        self.csv = open(self.out_dir / f'link_{int(time.time())}.csv', 'w', buffering=1)
        self.csv.write('t,launcher,drone,pose_hz,sent,sent_ping,receive,enqueued,ack,'
                       'rx_uc,rx_bc\n')

        for n in names:
            self.create_subscription(PoseStamped, f'/{n}/pose',
                                     self._pose_cb(n), qos_profile_sensor_data)
            self._subscribe_stats(n)
            self._subscribe_status(n)
        self.create_timer(1.0, self.tick)
        print(f'recording {len(names)} drones -> {self.out_dir}  '
              f'(launcher: {self.launcher})', flush=True)

    def _subscribe_stats(self, name):
        """Subscribe to the link counters if the message type is available."""
        try:
            from crazyflie_interfaces.msg import ConnectionStatisticsArray
        except ImportError:
            if name == self.names[0]:
                print('  NOTE: crazyflie_interfaces.ConnectionStatisticsArray not '
                      'found; recording pose rates only', flush=True)
            return

        def cb(msg, nm=name):
            if msg.stats:
                s = msg.stats[0]
                self.stats[nm] = dict(sent=s.sent_count, sent_ping=s.sent_ping_count,
                                      receive=s.receive_count,
                                      enqueued=s.enqueued_count, ack=s.ack_count)
        self.create_subscription(ConnectionStatisticsArray,
                                 f'/{name}/connection_statistics', cb, 10)

    def _subscribe_status(self, name):
        """Record the drone's OWN receive counters -- see the module docstring."""
        try:
            from crazyflie_interfaces.msg import Status
        except ImportError:
            return

        def cb(msg, nm=name):
            self.rx[nm] = dict(rx_uc=getattr(msg, 'num_rx_unicast', None),
                               rx_bc=getattr(msg, 'num_rx_broadcast', None))
        self.create_subscription(Status, f'/{name}/status', cb, 10)

    def _pose_cb(self, name):
        def cb(_msg):
            now = time.time()
            self.stamps[name].append(now)
            self.last_pose[name] = now
        return cb

    def rate(self, name, now):
        cut = now - self.window
        return sum(1 for t in self.stamps[name] if t >= cut) / self.window

    def tick(self):
        try:
            self._tick()
        except Exception as exc:                             # noqa: BLE001
            # An instrument that dies at the moment of interest is worse than
            # no instrument: keep recording whatever happens next.
            print(f'  recorder tick error (continuing): {exc!r}', flush=True)

    def _tick(self):
        now = time.time()
        for n in self.names:
            hz = self.rate(n, now)
            st = self.stats[n] or {}
            rx = self.rx[n] or {}
            row = dict(t=round(now, 2), pose_hz=round(hz, 2), **st, **rx)
            self.history[n].append(row)
            self.csv.write(
                f'{now:.2f},{self.launcher},{n},{hz:.2f},'
                f'{st.get("sent","")},{st.get("sent_ping","")},'
                f'{st.get("receive","")},{st.get("enqueued","")},{st.get("ack","")},'
                f'{rx.get("rx_uc","")},{rx.get("rx_bc","")}\n')

            # A GAP, not a windowed rate: the watchdog recovers a stalled
            # drone in ~1 s, which a 5 s window never sees as zero.
            last = self.last_pose[n]
            gap = None if last is None else now - last

            if hz >= ALIVE_HZ:
                self.was_alive[n] = True
            if gap is not None and gap < STALL_GAP_S and n in self.stalled:
                self.stalled.discard(n)
                print(f'  [{time.strftime("%H:%M:%S")}] {n} RECOVERED after '
                      f'{gap:.1f}s gap ({hz:.1f} Hz) - note what changed',
                      flush=True)
            elif (gap is not None and gap >= STALL_GAP_S
                    and self.was_alive[n] and n not in self.stalled):
                # Every drone quiet at once is the stack being stopped, not
                # five simultaneous stalls -- do not write junk dumps for it.
                others = [m for m in self.names if m != n]
                if others and all(
                        self.last_pose[m] is None
                        or now - self.last_pose[m] >= STALL_GAP_S
                        for m in others):
                    print(f'  [{time.strftime("%H:%M:%S")}] all drones quiet '
                          '- treating as a stack shutdown, not a stall',
                          flush=True)
                    self.was_alive[n] = False
                    continue
                self.stalled.add(n)
                print(f'  [{time.strftime("%H:%M:%S")}] {n} STALLED '
                      f'({gap:.1f}s with no pose) - dumping in '
                      f'{DUMP_DELAY_S:.0f}s so the recovery is in the window',
                      flush=True)
                self.pending_dump[n] = now + DUMP_DELAY_S

        # Deferred dumps, so the watchdog's recovery is inside the precursor.
        for n, when in list(self.pending_dump.items()):
            if now >= when:
                del self.pending_dump[n]
                self.dump(n, now)

    def dump(self, victim, now):
        """Write the precursor window for every drone, plus the server log tail."""
        path = self.out_dir / f'stall_{victim}_{time.strftime("%Y%m%d-%H%M%S")}.json'
        logdir = sorted(pathlib.Path(os.path.expanduser('~/.ros/log')).glob(
            'crazyflie_server_*.log'), key=lambda p: p.stat().st_mtime)
        tail = []
        if logdir:
            try:
                with open(logdir[-1], errors='replace') as f:
                    tail = f.readlines()[-400:]
            except OSError:
                pass
        payload = dict(
            victim=victim, when=now, launcher=self.launcher,
            window_s=self.window, precursor_s=PRECURSOR_S,
            note=('rx_uc/rx_bc are what the DRONE received (healthy ~170/~145); '
                  'a second at ~0 is the firmware 1 s radio-activity timeout '
                  'that makes it delete its own log blocks. Server-side: sent '
                  'flat = not polled; ack flat = no acks'),
            drones={n: list(self.history[n]) for n in self.names},
            still_alive=[n for n in self.names if n not in self.stalled],
            server_log_tail=tail)
        path.write_text(json.dumps(payload, indent=1))
        print(f'\n  *** [{time.strftime("%H:%M:%S")}] {victim} STALLED -- '
              f'wrote {path.name}', flush=True)
        # The counters are PER-SECOND snapshots, not cumulative totals, so
        # differencing them is meaningless (an earlier version printed negative
        # "deltas"). Print the raw trailing samples instead: what matters is
        # whether sent/ack stay HIGH while pose_hz goes to zero -- that is the
        # signature that rules out starvation and the safelink discard.
        rows = [r for r in self.history[victim] if 'sent' in r][-6:]
        if rows:
            print('      per-second counters into the stall '
                  '(pose_hz | sent | ack | DRONE rx_uc | rx_bc):', flush=True)
            for r in rows:
                print(f'        {r.get("pose_hz"):>5} | {r.get("sent"):>4} | '
                      f'{r.get("ack"):>4} | {str(r.get("rx_uc")):>6} | '
                      f'{str(r.get("rx_bc")):>5}', flush=True)
        else:
            print('      no link counters (set warnings.communication.'
                  'publish_stats: true in server.yaml and relaunch)', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', default=str(WORKSPACE / 'data/linkstalls'))
    ap.add_argument('--window', type=float, default=5.0,
                    help='seconds used to compute each pose rate (default 5)')
    ap.add_argument('--yaml', default=str(FLEET_YAML))
    args = ap.parse_args()

    names = enabled_drones(args.yaml)
    if not names:
        sys.exit(f'no enabled drones in {args.yaml}')
    rclpy.init()
    node = Recorder(names, args.out, args.window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nstopped.')
    finally:
        node.csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
