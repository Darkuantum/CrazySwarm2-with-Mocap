"""What is actually running on this machine -- whoever started it.

The console used to know only its own children. A stack launched from a
terminal, a show started by hand, a test stack Claude brought up on another
ROS_DOMAIN_ID: all invisible -- yet they hold the same Crazyradio and the same
UDP 1511, so "is the radio free?" and "may I launch?" were answered from half
the picture. This module is the other half: a census of /proc, every sweep,
of the processes that make up this rig.

For each one: its role (launch / server / mocap / bridge / rviz / preflight /
script), its PID and process group, its ROS_DOMAIN_ID (read from
/proc/<pid>/environ -- a process on another domain is invisible to every ros2
CLI probe, which is exactly why it has to be found this way), when it started,
and who started it -- this console (its process group is one the console
created) or something outside it.

Read-only: it reads /proc. Stopping an outside process goes through
`signal_group`, which the backend only calls on an explicit, confirmed request.
"""

from __future__ import annotations

import os
import re
import signal
import time

from . import builds, catalog

_CLK = os.sysconf('SC_CLK_TCK')
_BOOT = None


def _boot_time():
    global _BOOT
    if _BOOT is None:
        try:
            with open('/proc/stat') as fh:
                for ln in fh:
                    if ln.startswith('btime '):
                        _BOOT = float(ln.split()[1])
        except OSError:
            _BOOT = 0.0
    return _BOOT


def _read(pid, name):
    try:
        with open(f'/proc/{pid}/{name}', 'rb') as fh:
            return fh.read()
    except OSError:
        return None


def _stat(pid):
    raw = _read(pid, 'stat')
    if not raw:
        return None
    s = raw.decode('utf-8', 'replace')
    rest = s[s.rfind(')') + 2:].split()
    # fields after "(comm)": state ppid pgrp session ... starttime is field 22
    try:
        return {'state': rest[0], 'ppid': int(rest[1]), 'pgid': int(rest[2]),
                'started': _boot_time() + int(rest[19]) / _CLK}
    except (IndexError, ValueError):
        return None


def _domain(pid):
    raw = _read(pid, 'environ')
    if raw is None:
        return None                       # another user's process: unknown
    for kv in raw.split(b'\0'):
        if kv.startswith(b'ROS_DOMAIN_ID='):
            return kv.split(b'=', 1)[1].decode() or '0'
    return '0'


_LAUNCH_ARG = re.compile(r'^([A-Za-z_]\w*):=(.*)$')


def _classify(argv, script_exes):
    """-> (role, detail) or None. argv is the process's own argv."""
    if not argv:
        return None
    joined = ' '.join(argv)
    base = [os.path.basename(a) for a in argv[:3]]
    if 'ros2' in base and 'launch' in argv[:4] and any(a in ('crazyflie', 'crazyflie_shows') for a in argv):
        i = argv.index('launch')
        args = dict(m.groups() for m in (_LAUNCH_ARG.match(a) for a in argv[i + 1:]) if m)
        return 'launch', {'file': ' '.join(argv[i + 1:i + 3]), 'args': args,
                          'backend': args.get('backend', 'cpp'),
                          'server': args.get('server', 'True').lower() in ('true', '1')}
    for a in argv[:2]:
        if a.endswith('/crazyflie_server') or a.endswith('crazyflie_server.py'):
            backend = 'sim' if '/crazyflie_sim/' in a else ('cflib' if a.endswith('.py') else 'cpp')
            return 'server', {'backend': backend}
        if a.endswith('/motion_capture_tracking_node'):
            return 'mocap', {}
        if a.endswith('/foxglove_bridge'):
            return 'bridge', {}
        if a.endswith('/rviz2'):
            return 'rviz', {}
        if a.endswith('preflight_kalman_plotter.py'):
            return 'preflight', {}
    # a flight/ground script: install/<pkg>/lib/<pkg>/<exe>, run by its python
    for a in argv[:2]:
        m = re.search(r'/install/([^/]+)/lib/\1/([^/]+)$', a)
        if m and (m.group(1), m.group(2)) in script_exes:
            pkg, exe = m.groups()
            params = dict(x.split(':=', 1) for x in argv if ':=' in x and not x.startswith('__'))
            return 'script', {'pkg': pkg, 'exe': exe, 'ground': catalog._is_ground(exe),
                              'params': params}
    if 'ros2' in base and 'run' in argv[:4]:
        return None                       # the `ros2 run` wrapper: its child is the script
    del joined
    return None


def _script_exes():
    out = set()
    for st in builds.status():
        for e in st['entry_points']:
            out.add((st['pkg'], e))
    return out


def census(console_pgids=None, my_domain=None):
    """Every rig process on the machine.

    console_pgids: {pgid: console proc id} for the console's own children
    (each is started in a new session, so its pgid is the child's pid).
    """
    console_pgids = console_pgids or {}
    my_domain = my_domain if my_domain is not None else os.environ.get('ROS_DOMAIN_ID', '0') or '0'
    exes = _script_exes()
    me = os.getpid()
    out = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == me:
            continue
        raw = _read(pid, 'cmdline')
        if not raw:
            continue
        argv = [a.decode('utf-8', 'replace') for a in raw.rstrip(b'\0').split(b'\0')]
        hit = _classify(argv, exes)
        if not hit:
            continue
        st = _stat(pid)
        if not st or st['state'] == 'Z':
            continue
        role, detail = hit
        dom = _domain(pid)
        out.append({
            'pid': pid, 'pgid': st['pgid'], 'ppid': st['ppid'], 'role': role,
            'started': st['started'], 'domain': dom,
            'other_domain': dom is not None and dom != my_domain,
            'by': 'console' if st['pgid'] in console_pgids else 'outside',
            'console_proc': console_pgids.get(st['pgid']),
            'cmdline': ' '.join(argv)[:400], **detail,
        })
    out.sort(key=lambda p: p['started'])
    return out


def summarise(procs, my_domain):
    """The facts the rest of the console reasons with."""
    by = lambda role: [p for p in procs if p['role'] == role]
    servers, mocaps, launches = by('server'), by('mocap'), by('launch')
    scripts = by('script')
    flights = [p for p in scripts if not p['ground']]
    domains = sorted({p['domain'] for p in procs if p['domain'] is not None})
    stack = None
    if launches or servers or mocaps:
        lead = (launches or servers or mocaps)[-1]
        stack = {
            'pid': lead['pid'], 'by': lead['by'], 'domain': lead['domain'],
            'backend': (servers[-1]['backend'] if servers else lead.get('backend')),
            'server': bool(servers), 'mocap': bool(mocaps),
            'args': lead.get('args', {}), 'started': lead['started'],
        }
    return {
        'stack': stack,
        'servers': len(servers),
        'server_any': bool(servers),           # on ANY domain: owns the radio
        'mocap_any': bool(mocaps),             # on ANY domain: holds UDP 1511
        'stack_any': bool(launches or servers or mocaps),
        'flights': [{'pid': p['pid'], 'exe': p['exe'], 'pkg': p['pkg'], 'by': p['by'],
                     'domain': p['domain'], 'console_proc': p['console_proc']} for p in flights],
        'outside': [p for p in procs if p['by'] == 'outside'],
        'other_domains': [d for d in domains if d != my_domain],
        'my_domain': my_domain,
    }


def signal_group(pgid, sig=signal.SIGINT):
    """SIGINT (default) to an outside process group. Only on an explicit,
    confirmed request from the operator -- see server.py."""
    try:
        os.killpg(pgid, sig)
        return True
    except OSError:
        return False


def now():
    return time.time()
