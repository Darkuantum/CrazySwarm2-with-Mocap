"""The fleet-command journal: every command the server ACTED on, from anyone.

The console's own history knows what the console sent. It cannot know about
the preflight GUI's `e` key, a `ros2 service call` in a terminal, a show's
own landing, or a command Claude sent while investigating. The server knows
all of them, because it logs each one as it handles it --

    [all] emergency()                                  crazyflie_server.cpp
    [cf3] takeoff(height=0.500000 m, duration=...)     crazyflie_server.cpp
    [all] emergency not yet implemented                crazyflie_sim (!)

-- to /rosout. So this reads /rosout, through the same CLI an operator would
type (`ros2 topic echo /rosout --csv`; the CLI-only rule holds), keeps the
lines from crazyflie_server that are commands, and publishes each as a
`fleet` event. An e-stop from ANY source therefore reaches every console page.

It sees the console's own ROS_DOMAIN_ID only -- a server on another domain is
reported by the process census (system.py) instead, which is how the console
can say "there is a stack you cannot hear".

/rosout is TRANSIENT_LOCAL: on (re)connect the server's recent backlog
arrives too. Those events are kept but marked `backlog`, so an old e-stop is
history, not an alarm.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import threading
import time
from collections import deque

CMD = ['ros2', 'topic', 'echo', '/rosout', '--csv', '--qos-durability', 'transient_local']
SERVER_NODES = ('crazyflie_server',)
_COMMAND = re.compile(
    r'^\[(?P<target>[\w/]+)\]\s+(?P<verb>emergency|takeoff|land|arm|go_to|goto|'
    r'upload_trajectory|upload trajectory|start_trajectory|start trajectory|'
    r'notify_setpoints_stop|notify setpoint stop)\b(?P<rest>.*)$', re.I)


def parse_csv_line(line):
    """`ros2 topic echo /rosout --csv` -> dict, or None.

    rcl_interfaces/Log flattens to: sec, nanosec, level, name, msg, file,
    function, line. msg may itself contain commas, so it is everything between
    the 4th comma and the 3rd-from-last.
    """
    parts = line.rstrip('\n').split(',')
    if len(parts) < 8:
        return None
    try:
        ts = int(parts[0]) + int(parts[1]) * 1e-9
        level = int(parts[2])
    except ValueError:
        return None
    return {'ts': ts, 'level': level, 'node': parts[3],
            'msg': ','.join(parts[4:-3]).strip(), 'file': parts[-3]}


def classify(rec):
    """A /rosout record -> fleet event dict, or None if it is not a command."""
    if rec is None or rec['node'].rsplit('/', 1)[-1] not in SERVER_NODES:
        return None
    m = _COMMAND.match(rec['msg'])
    if not m:
        return None
    verb = m.group('verb').lower().replace(' ', '_')
    verb = {'goto': 'go_to'}.get(verb, verb)
    rest = m.group('rest')
    sim_noop = 'not yet implemented' in rest
    return {'ts': rec['ts'], 'target': m.group('target'), 'verb': verb,
            'detail': rest.strip(' ()'), 'sim_noop': sim_noop,
            'level': rec['level'], 'text': rec['msg']}


class Journal:
    def __init__(self, bus, history=300):
        self.bus = bus
        self.events = deque(maxlen=history)
        self.started = time.time()
        self.state = 'starting'
        self.error = ''
        self._seen = set()
        self._stop = False
        if shutil.which('ros2'):
            threading.Thread(target=self._loop, daemon=True).start()
        else:
            self.state, self.error = 'off', 'ros2 is not on PATH'

    @property
    def cmdline(self):
        return ' '.join(CMD)

    def _loop(self):
        while not self._stop:
            self.state = 'listening'
            try:
                proc = subprocess.Popen(CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        text=True, bufsize=1,
                                        env={**os.environ, 'PYTHONUNBUFFERED': '1'})
                self.proc = proc
                for line in proc.stdout:
                    ev = classify(parse_csv_line(line))
                    if ev is None:
                        continue
                    key = (round(ev['ts'], 6), ev['text'])
                    if key in self._seen:             # backlog replayed on reconnect
                        continue
                    self._seen.add(key)
                    ev['backlog'] = ev['ts'] < self.started - 1.0
                    self.events.append(ev)
                    self.bus.publish({'type': 'fleet', 'event': ev})
                proc.wait()
            except Exception as exc:                      # noqa: BLE001 - never kill the console
                self.error = str(exc)
            self.state = 'restarting'
            time.sleep(3.0)

    def recent(self, n=100):
        return list(self.events)[-n:]

    def last_estop(self):
        for ev in reversed(self.events):
            if ev['verb'] == 'emergency':
                return ev
        return None

    def stop(self):
        self._stop = True
        p = getattr(self, 'proc', None)
        if p and p.poll() is None:
            p.terminate()
