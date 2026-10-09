"""HTTP + SSE backend for the mission console.

Stdlib only. The UI is a single page served from static/; everything it does
goes through the small JSON API below, and every API call that runs something
records the exact command line in the history so the session can be exported as
a plain shell script.
"""

from __future__ import annotations

import glob
import json
import mimetypes
import os
import shlex
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import yaml

from . import builds, catalog, configio, missions, usage
from .health import Health
from .procs import EventBus, ProcessManager, is_flight

STATIC = os.path.join(os.path.dirname(__file__), 'static')
REPO = configio.REPO
# Two cadences. The full sweep is the slow, complete picture; the live sweep
# re-runs only what moves while you watch (ROS graph, /poses, per-drone battery
# / link / supervisor state) and carries the rest over. Tiles were up to ~24 s
# stale on the single 20 s cadence, which is a long time to look at a drone and
# not know it is e-stopped.
HEALTH_PERIOD = 30.0        # full sweep
LIVE_PERIOD = 3.0           # live sweep


class App:
    def __init__(self):
        self.bus = EventBus()
        self.procs = ProcessManager(self.bus, REPO)
        self.health = Health(self.procs)
        self.history = []
        self._health_lock = threading.Lock()
        self._catalog_cache = (0.0, [])
        self._missions_cache = (None, [])
        self.started = time.time()
        threading.Thread(target=self._health_loop, daemon=True).start()

    # ------------------------------------------------------------- catalog
    def catalog(self):
        """Rebuilt when crazyflies.yaml changes (per-drone actions stay true) or
        when the set of installed scripts changes (a newly built show package
        appears without restarting the console)."""
        try:
            mtime = os.path.getmtime(configio.FILES['crazyflies']['path'])
        except OSError:
            mtime = 0.0
        key = (mtime, catalog.scripts_signature())
        if key != self._catalog_cache[0] or not self._catalog_cache[1]:
            cf = configio.load('crazyflies')
            fleet = configio.fleet_summary(cf['doc']) if cf['doc'] else []
            self._catalog_cache = (key, catalog.build_catalog(fleet))
        return self._catalog_cache[1]

    def action(self, action_id):
        for a in self.catalog():
            if a['id'] == action_id:
                return a
        raise KeyError(action_id)

    def labels(self):
        return {a['id']: a['label'] for a in self.catalog()}

    # ------------------------------------------------------------ missions
    def missions(self):
        """Re-read when a spec file changes or a package is (re)built, so a
        new mission appears on the next page load with no console restart."""
        key = (missions.signature(), catalog.scripts_signature(), builds.signature())
        if key != self._missions_cache[0]:
            self._missions_cache = (key, missions.discover())
        return self._missions_cache[1]

    def mission(self, mid):
        for m in self.missions():
            if m['id'] == mid:
                if m.get('error'):
                    raise missions.SpecError(f'{mid}: {m["error"]}')
                return m
        raise KeyError(mid)

    def mission_procs(self, mid):
        prefix = missions.action_id(mid, '')
        return [p.summary() for p in self.procs.procs.values()
                if p.action_id.startswith(prefix)]

    def start_mission(self, mid, role, values, sim):
        m = self.mission(mid)
        aid = missions.action_id(mid, role)
        if self.procs.find_running(aid):
            raise ValueError(f'{role} is already running for this mission')
        if role == 'main' and m['flight']:
            # two scripts commanding the same drones fight each other, and the
            # loser is whichever one the drones obey less
            other = next((p for p in self.procs.procs.values()
                          if p.running and is_flight(p.action_id)), None)
            if other:
                raise ValueError(f'another flight script is running ({other.label}) -- '
                                 'stop it first: two scripts must never command the drones at once')
        bs = missions.build_state(m)
        if bs['problems']:
            raise ValueError('not built: ' + '; '.join(bs['problems']) +
                             f' -- run {bs["build_cmd"]}, then restart the console')
        if bs['restart_console']:
            raise ValueError('built after this console started, so `ros2 run` from here '
                             'cannot see it -- restart ./console/run.sh')
        argv, cmdline, label = missions.build(m, role, values, sim=sim)
        proc = self.procs.start(aid, label, argv, kind='service' if role != 'main' else 'task',
                                note=m['summary'])
        entry = {'ts': time.time(), 'action_id': aid, 'label': label,
                 'cmdline': cmdline, 'proc': proc.id}
        self.history.append(entry)
        self.bus.publish({'type': 'history', 'entry': entry})
        usage.record('mission_start', mission=mid, role=role, sim=sim,
                     values={k: v for k, v in (values or {}).items() if k != 'extra'})
        threading.Timer(2.0, self.refresh_health).start()
        return proc.summary()

    # -------------------------------------------------------------- health
    def refresh_health(self, full=True):
        with self._health_lock:
            try:
                snap = self.health.run(full=full)
            except Exception:                            # noqa: BLE001
                snap = {'nodes': [], 'edges': [], 'facts': {}, 'worst': 'unknown',
                        'headline': 'health check crashed',
                        'error': traceback.format_exc(), 'ts': time.time()}
                self.health.last = snap
            self.bus.publish({'type': 'health', 'health': snap})
            return snap

    def _health_loop(self):
        time.sleep(1.0)
        last_full = 0.0
        while True:
            try:
                # the first sweep of the process, and every HEALTH_PERIOD after,
                # is a full one; the rest are live. Anything you DO -- an action,
                # a config write, Re-check -- still forces a full sweep, so an
                # edit is never waiting on this timer.
                full = time.time() - last_full >= HEALTH_PERIOD
                self.refresh_health(full=full)
                if full:
                    last_full = time.time()
            except Exception:                            # noqa: BLE001
                pass
            time.sleep(LIVE_PERIOD)

    # ---------------------------------------------------------------- runs
    def run_action(self, action_id, values, source=None):
        act = self.action(action_id)
        argv, cmdline = catalog.render(act, values)
        usage.record('run', action=action_id, source=source, values=values or None)
        proc = self.procs.start(action_id, act['label'], argv,
                                kind=act.get('kind', 'task'), note=act.get('why', ''))
        entry = {'ts': time.time(), 'action_id': action_id, 'label': act['label'],
                 'cmdline': cmdline, 'proc': proc.id}
        self.history.append(entry)
        self.bus.publish({'type': 'history', 'entry': entry})
        threading.Timer(2.0, self.refresh_health).start()
        return proc.summary()

    # ------------------------------------------------------------- e-stop
    # An e-stop must be one action with an answer. The generic run path gives
    # neither: it sits behind a confirm modal, and `ros2 service call` has no
    # timeout -- if /all/emergency is unreachable (server dead or hung, the
    # console on a different ROS_DOMAIN_ID than the server, discovery stalled)
    # it waits forever and prints NOTHING, so the operator cannot tell whether
    # the motors were cut. Measured: domain mismatch -> silent, never returns.
    ESTOP_CONFIRM_S = 3.0    # past this the UI says NOT CONFIRMED, loudly
    ESTOP_ABANDON_S = 15.0   # then stop waiting: a call left pending would
                             # e-stop a server launched LATER, by surprise

    def estop(self, source=None):
        usage.record('estop', source=source)
        act = self.action('srv.estop')
        argv, cmdline = catalog.render(act, {})
        t0 = time.time()
        proc = self.procs.start('srv.estop', act['label'], argv, kind='task',
                                note=act.get('why', ''))
        entry = {'ts': t0, 'action_id': 'srv.estop', 'label': act['label'],
                 'cmdline': cmdline, 'proc': proc.id}
        self.history.append(entry)
        self.bus.publish({'type': 'history', 'entry': entry})

        while proc.running and time.time() - t0 < self.ESTOP_CONFIRM_S:
            time.sleep(0.02)
        elapsed = time.time() - t0
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        out = {'proc': proc.id, 'cmdline': cmdline, 'elapsed': elapsed,
               'domain': domain, 'rc': proc.returncode}

        if proc.returncode == 0:
            out.update(confirmed=True, pending=False,
                       reason='The crazyflie_server accepted /all/emergency.')
        elif proc.running:
            # Keep the attempt alive a little longer in case discovery is only
            # slow -- a late success is still a success -- but not forever.
            def _abandon(p=proc):
                if p.running:
                    self.procs.stop(p.id, hard=True)
            threading.Timer(max(0.0, self.ESTOP_ABANDON_S - elapsed), _abandon).start()
            out.update(confirmed=False, pending=True, reason=(
                f'No answer from /all/emergency within {self.ESTOP_CONFIRM_S:.0f} s on '
                f'ROS_DOMAIN_ID={domain}. The server is down, hung, or on a different '
                f'domain than this console. Still retrying for '
                f'{self.ESTOP_ABANDON_S:.0f} s -- do not wait for it: use the preflight '
                f'GUI (e) or cut power.'))
        else:
            tail = [r[2] for r in proc.tail(limit=6)]
            out.update(confirmed=False, pending=False, reason=(
                f'`{cmdline}` exited {proc.returncode}: ' + ' | '.join(tail[-3:])))
        self.refresh_health_soon()
        return out

    def refresh_health_soon(self):
        threading.Timer(0.5, self.refresh_health).start()

    def history_script(self):
        lines = [
            '#!/usr/bin/env bash',
            '# Commands run from the CrazySwarm2 mission console.',
            '# Prelude (what console/run.sh does for you):',
            '#   source /opt/ros/$ROS_DISTRO/setup.bash',
            '#   source install/setup.bash',
            f'cd {shlex.quote(REPO)}',
            '',
        ]
        for e in self.history:
            lines.append('# ' + time.strftime('%H:%M:%S', time.localtime(e['ts'])) +
                         '  ' + e['label'])
            lines.append(e['cmdline'])
            lines.append('')
        return '\n'.join(lines)


APP = App()


class Handler(BaseHTTPRequestHandler):
    server_version = 'CrazyswarmConsole/1.0'
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):           # quiet: the UI is the log
        pass

    # ------------------------------------------------------------ plumbing
    def _send(self, code, body=b'', ctype='application/json', extra=None):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, default=str), 'application/json')

    def _body(self):
        length = int(self.headers.get('Content-Length') or 0)
        if not length:
            return {}
        return json.loads(self.rfile.read(length) or b'{}')

    # ---------------------------------------------------------------- GET
    def do_GET(self):
        url = urlparse(self.path)
        path, query = url.path, parse_qs(url.query)
        try:
            if path == '/' or path == '/index.html':
                return self._static('index.html')
            if path.startswith('/static/'):
                return self._static(path[len('/static/'):])
            if path == '/api/bootstrap':
                return self._json({
                    'code_changed': _code_changed(),
                    'catalog': catalog.public(APP.catalog()),
                    'groups': _groups(APP.catalog()),
                    'files': [{'key': k, 'label': v['label'], 'blurb': v['blurb'],
                               'path': os.path.relpath(v['path'], REPO)}
                              for k, v in configio.FILES.items()],
                    'health': APP.health.last or {'nodes': [], 'edges': [], 'facts': {},
                                                  'headline': 'checking...', 'worst': 'unknown'},
                    'procs': APP.procs.list(),
                    'history': APP.history,
                    'repo': REPO,
                    'missions': _mission_cards(),
                    'prefs': usage.load_prefs(),
                    'usage': usage.summary(days=30, labels=APP.labels()),
                    'env': {
                        'ros_distro': os.environ.get('ROS_DISTRO', ''),
                        'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                    },
                })
            if path == '/api/health':
                if query.get('refresh'):
                    return self._json(APP.refresh_health())
                return self._json(APP.health.last or {})
            if path == '/api/procs':
                return self._json({'procs': APP.procs.list()})
            if path.startswith('/api/proc/'):
                pid = path.rsplit('/', 1)[-1]
                proc = APP.procs.get(pid)
                if proc is None:
                    return self._json({'error': 'no such process'}, 404)
                after = int((query.get('after') or ['0'])[0])
                return self._json({'proc': proc.summary(),
                                   'lines': [{'seq': s, 'ts': t, 'text': x}
                                             for s, t, x in proc.tail(after)]})
            if path.startswith('/api/config/'):
                key = path.rsplit('/', 1)[-1]
                if key not in configio.FILES:
                    return self._json({'error': 'unknown file'}, 404)
                data = configio.load(key)
                extra = {}
                if key == 'crazyflies' and data['doc']:
                    extra = {'fleet': configio.fleet_summary(data['doc']),
                             'findings': configio.validate_crazyflies(data['doc']),
                             'types': sorted((data['doc'].get('robot_types') or {}).keys())}
                elif key == 'motion_capture' and data['doc']:
                    extra = {'findings': configio.validate_motion_capture(data['doc'])}
                return self._json({**{k: v for k, v in data.items() if k != 'doc'}, **extra})
            if path in ('/mission', '/mission/'):
                return self._static('mission/index.html')
            if path.startswith('/mission-assets/'):
                return self._mission_asset(path[len('/mission-assets/'):])
            if path == '/api/missions':
                st = builds.status()
                return self._json({'missions': _mission_cards(st),
                                   'unviewed': missions.unviewed(APP.missions(), st),
                                   'packages': st})
            if path == '/api/mission':
                mid = (query.get('id') or [''])[0]
                try:
                    m = APP.mission(mid)
                except KeyError:
                    return self._json({'error': f'no mission {mid!r}'}, 404)
                except missions.SpecError as exc:
                    return self._json({'error': str(exc)}, 422)
                return self._json({'mission': missions.public(m),
                                   'build': missions.build_state(m),
                                   'procs': APP.mission_procs(mid)})
            if path == '/api/arena':
                return self._json(_arena())
            if path == '/api/usage':
                days = (query.get('days') or [''])[0]
                return self._json(usage.summary(days=float(days) if days else None,
                                                labels=APP.labels()))
            if path == '/api/prefs':
                return self._json(usage.load_prefs())
            if path == '/api/history.sh':
                return self._send(200, APP.history_script(), 'text/plain; charset=utf-8')
            if path == '/api/events':
                return self._events()
            return self._json({'error': 'not found'}, 404)
        except Exception:                                # noqa: BLE001
            return self._json({'error': traceback.format_exc()}, 500)

    # --------------------------------------------------------------- POST
    def do_POST(self):
        path = urlparse(self.path).path
        try:
            body = self._body()
            if path == '/api/preview':
                act = APP.action(body['action_id'])
                argv, cmdline = catalog.render(act, body.get('values') or {})
                return self._json({'argv': argv, 'cmdline': cmdline})
            if path == '/api/estop':
                return self._json(APP.estop(source=body.get('source')))
            if path == '/api/run':
                return self._json(APP.run_action(body['action_id'], body.get('values') or {},
                                                 source=body.get('source')))
            if path == '/api/stop':
                proc = APP.procs.get(body['proc'])
                mode = body.get('mode') or ('kill' if body.get('hard') else 'stop')
                usage.record('stop', action=proc.action_id if proc else None, mode=mode,
                             source=body.get('source'))
                if mode == 'interrupt':
                    return self._json({'ok': APP.procs.interrupt(body['proc'])})
                return self._json({'ok': APP.procs.stop(body['proc'], mode == 'kill')})
            if path == '/api/input':
                if body.get('source') != 'teleop':      # keystrokes are not logged one by one
                    proc = APP.procs.get(body['proc'])
                    usage.record('input', action=proc.action_id if proc else None,
                                 shape=usage.stdin_shape(body['text']), source=body.get('source'))
                return self._json({'ok': APP.procs.send_input(body['proc'], body['text'])})
            if path == '/api/usage':
                ev = body.get('ev')
                if ev in usage.CLIENT_EVENTS:
                    usage.record(ev, **{k: v for k, v in body.items()
                                        if k != 'ev' and isinstance(v, (str, int, float, bool))})
                return self._json({'ok': True})
            if path == '/api/prefs':
                return self._json(usage.save_prefs(body))
            if path == '/api/mission/preview':
                m = APP.mission(body['id'])
                argv, cmdline, label = missions.build(m, body.get('role') or 'main',
                                                      body.get('values') or {},
                                                      sim=bool(body.get('sim')))
                return self._json({'argv': argv, 'cmdline': cmdline, 'label': label,
                                   'helpers': [h['key'] for h in missions.active_helpers(
                                       m, body.get('values') or {})]})
            if path == '/api/mission/start':
                return self._json(APP.start_mission(body['id'], body.get('role') or 'main',
                                                    body.get('values') or {},
                                                    bool(body.get('sim'))))
            if path == '/api/prune':
                APP.procs.prune()
                return self._json({'procs': APP.procs.list()})
            if path == '/api/scan_fleet':
                cf = configio.load('crazyflies')
                fleet = configio.fleet_summary(cf['doc'])
                usage.record('scan')
                result = APP.health.scan_fleet(fleet)
                entry = {'ts': time.time(), 'action_id': 'health.scan_fleet',
                         'label': 'Scan the fleet',
                         'cmdline': '; '.join(v['cmd'] for v in result.values()),
                         'proc': None}
                APP.history.append(entry)
                APP.bus.publish({'type': 'history', 'entry': entry})
                return self._json({'scan': result, 'health': APP.refresh_health()})
            if path == '/api/config/raw':
                return self._json(_write_raw(body))
            if path == '/api/config/robots':
                return self._json(_edit_robots(body))
            if path == '/api/config/kv':
                return self._json(_edit_kv(body))
            return self._json({'error': 'not found'}, 404)
        except ValueError as exc:
            # A rejected parameter field is the operator's typo, not a
            # crash. /api/preview runs on every keystroke, so this is the
            # text they read while typing -- a traceback there is noise
            # that hides it.
            return self._json({'error': str(exc)}, 400)
        except Exception:                                # noqa: BLE001
            return self._json({'error': traceback.format_exc()}, 500)

    # ----------------------------------------------------------- SSE/static
    def _events(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        sub = APP.bus.subscribe()
        try:
            self.wfile.write(b': connected\n\n')
            self.wfile.flush()
            while True:
                event = sub.get(timeout=15.0)
                if event is None:
                    self.wfile.write(b': ping\n\n')
                else:
                    payload = json.dumps(event, default=str)
                    self.wfile.write(f'data: {payload}\n\n'.encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            APP.bus.unsubscribe(sub)

    def _mission_asset(self, rest):
        # /mission-assets/<pkg>/<name>/<file>: a custom view module shipped
        # next to a mission spec. Path-checked to that spec's directory.
        parts = rest.split('/', 2)
        if len(parts) != 3:
            return self._json({'error': 'not found'}, 404)
        try:
            m = APP.mission(parts[0] + '/' + parts[1])
        except (KeyError, missions.SpecError):
            return self._json({'error': 'not found'}, 404)
        full = None if m.get('builtin') else missions.asset_path(m, parts[2])
        if not full:
            return self._json({'error': 'not found'}, 404)
        ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
        if full.endswith('.mjs'):
            ctype = 'text/javascript'
        if ctype.startswith('text/') or ctype == 'application/javascript':
            ctype += '; charset=utf-8'
        with open(full, 'rb') as fh:
            return self._send(200, fh.read(), ctype)

    def _static(self, rel):
        rel = rel.lstrip('/')
        full = os.path.normpath(os.path.join(STATIC, rel))
        if not full.startswith(STATIC) or not os.path.isfile(full):
            return self._json({'error': 'not found'}, 404)
        ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
        if full.endswith('.mjs'):
            ctype = 'text/javascript'      # ES modules are refused under any other type
        if ctype.startswith('text/') or ctype in ('application/javascript',):
            ctype += '; charset=utf-8'
        with open(full, 'rb') as fh:
            return self._send(200, fh.read(), ctype)


def _groups(acts):
    out = []
    for a in acts:
        if a['group'] not in out:
            out.append(a['group'])
    return out


def _write_raw(body):
    key = body['file']
    spec = configio.FILES[key]
    text = body['text']
    try:
        yaml.safe_load(text)
    except yaml.YAMLError as exc:
        return {'ok': False, 'error': f'YAML did not parse, nothing was written:\n{exc}'}
    if not body.get('apply'):
        import difflib
        with open(spec['path']) as fh:
            before = fh.read()
        return {'ok': True, 'preview': True,
                'diff': ''.join(difflib.unified_diff(
                    before.splitlines(keepends=True), text.splitlines(keepends=True),
                    fromfile=spec['label'] + ' (current)', tofile=spec['label'] + ' (new)'))}
    bak, diff = configio.write(spec['path'], text)
    usage.record('config_write', file=key, how='raw')
    APP.bus.publish({'type': 'config', 'file': key})
    threading.Timer(0.5, APP.refresh_health).start()
    return {'ok': True, 'written': bool(bak), 'backup': bak, 'diff': diff}


def _edit_robots(body):
    diff, text = configio.apply_robot_edits(
        body.get('edits') or [], body.get('adds') or [], body.get('removes') or [])
    try:
        yaml.safe_load(text)
    except yaml.YAMLError as exc:
        return {'ok': False, 'error': f'the edit produced invalid YAML, nothing written:\n{exc}'}
    if not body.get('apply'):
        return {'ok': True, 'preview': True, 'diff': diff}
    bak, applied = configio.write(configio.FILES['crazyflies']['path'], text)
    usage.record('config_write', file='crazyflies', how='fleet')
    APP.bus.publish({'type': 'config', 'file': 'crazyflies'})
    threading.Timer(0.5, APP.refresh_health).start()
    return {'ok': True, 'written': bool(bak), 'backup': bak, 'diff': applied}


def _edit_kv(body):
    key = body['file']
    diff, text = configio.apply_kv_edits(key, body.get('edits') or [])
    try:
        yaml.safe_load(text)
    except yaml.YAMLError as exc:
        return {'ok': False, 'error': f'the edit produced invalid YAML, nothing written:\n{exc}'}
    if not body.get('apply'):
        return {'ok': True, 'preview': True, 'diff': diff}
    bak, applied = configio.write(configio.FILES[key]['path'], text)
    usage.record('config_write', file=key, how='kv')
    APP.bus.publish({'type': 'config', 'file': key})
    return {'ok': True, 'written': bool(bak), 'backup': bak, 'diff': applied}


def _mission_cards(statuses=None):
    st = statuses if statuses is not None else builds.status()
    return [_mission_card(m, st) for m in APP.missions()]


def _mission_card(m, statuses):
    """The list entry for a mission: enough to show and open it -- and whether
    its scripts are built (a new show declared but not built says so here)."""
    return {'id': m['id'], 'pkg': m.get('pkg'), 'title': m.get('title'),
            'build': None if m.get('error') else missions.build_state(m, statuses),
            'summary': m.get('summary', ''), 'error': m.get('error'),
            'builtin': bool(m.get('builtin')),
            'flight': m.get('flight', True),
            'running': [p['id'] for p in APP.mission_procs(m['id'])
                        if p['state'] in ('running', 'stopping')]}


def _arena():
    """The room (arena.yaml) and the fleet's parking marks, for the 3D view.

    Same file the planners and shows read ($CRAZYSWARM_ARENA overrides it,
    exactly as in crazyflie_shows/safety.py). The view only DRAWS it -- it
    enforces nothing, so a missing file is reported, not fatal.
    """
    path = os.environ.get('CRAZYSWARM_ARENA') or os.path.join(configio.CFG_DIR, 'arena.yaml')
    out = {'path': os.path.relpath(path, REPO), 'arena': None, 'error': None}
    try:
        with open(path, encoding='utf-8') as fh:
            doc = yaml.safe_load(fh) or {}
        out['arena'] = doc.get('arena')
        out['separation'] = doc.get('separation')
    except (OSError, yaml.YAMLError) as exc:
        out['error'] = str(exc)
    cf = configio.load('crazyflies')
    out['fleet'] = configio.fleet_summary(cf['doc']) if cf['doc'] else []
    return out


def _code_changed():
    """True when console code on disk is newer than this running process.

    The HTML/JS/CSS are read from disk on every request, but this Python is
    loaded once at startup. Update the console without restarting it and the
    page calls endpoints this process has never heard of -- which answer
    {"error": "not found"}. That is how an e-stop press could come back as
    "not found" and fire nothing. The page shows a restart banner when this is
    set, before anyone needs a button.
    """
    newest = 0.0
    for f in glob.glob(os.path.join(os.path.dirname(__file__), '*.py')):
        try:
            newest = max(newest, os.path.getmtime(f))
        except OSError:
            pass
    return newest > APP.started


def serve(host='127.0.0.1', port=8077):
    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.daemon_threads = True
    return httpd
