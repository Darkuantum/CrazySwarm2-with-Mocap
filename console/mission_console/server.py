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

from . import catalog, configio
from .health import Health
from .procs import EventBus, ProcessManager

STATIC = os.path.join(os.path.dirname(__file__), 'static')
REPO = configio.REPO
HEALTH_PERIOD = 20.0


class App:
    def __init__(self):
        self.bus = EventBus()
        self.procs = ProcessManager(self.bus, REPO)
        self.health = Health(self.procs)
        self.history = []
        self._health_lock = threading.Lock()
        self._catalog_cache = (0.0, [])
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

    # -------------------------------------------------------------- health
    def refresh_health(self):
        with self._health_lock:
            try:
                snap = self.health.run()
            except Exception:                            # noqa: BLE001
                snap = {'nodes': [], 'edges': [], 'facts': {}, 'worst': 'unknown',
                        'headline': 'health check crashed',
                        'error': traceback.format_exc(), 'ts': time.time()}
                self.health.last = snap
            self.bus.publish({'type': 'health', 'health': snap})
            return snap

    def _health_loop(self):
        time.sleep(1.0)
        while True:
            try:
                self.refresh_health()
            except Exception:                            # noqa: BLE001
                pass
            time.sleep(HEALTH_PERIOD)

    # ---------------------------------------------------------------- runs
    def run_action(self, action_id, values):
        act = self.action(action_id)
        argv, cmdline = catalog.render(act, values)
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

    def estop(self):
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
                return self._json(APP.estop())
            if path == '/api/run':
                return self._json(APP.run_action(body['action_id'], body.get('values') or {}))
            if path == '/api/stop':
                return self._json({'ok': APP.procs.stop(body['proc'], bool(body.get('hard')))})
            if path == '/api/input':
                return self._json({'ok': APP.procs.send_input(body['proc'], body['text'])})
            if path == '/api/prune':
                APP.procs.prune()
                return self._json({'procs': APP.procs.list()})
            if path == '/api/scan_fleet':
                cf = configio.load('crazyflies')
                fleet = configio.fleet_summary(cf['doc'])
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

    def _static(self, rel):
        rel = rel.lstrip('/')
        full = os.path.normpath(os.path.join(STATIC, rel))
        if not full.startswith(STATIC) or not os.path.isfile(full):
            return self._json({'error': 'not found'}, 404)
        ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
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
    APP.bus.publish({'type': 'config', 'file': key})
    return {'ok': True, 'written': bool(bak), 'backup': bak, 'diff': applied}


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
