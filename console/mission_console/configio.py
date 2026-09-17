"""Read, validate and surgically edit the workspace's YAML config from the UI.

Writing rule: NEVER yaml.dump(). `crazyflies.yaml` and `motion_capture.yaml`
carry hard-won comments (the log-block byte budget, the channel-spacing rule,
the Motive 'auto' discovery note); a load/dump round-trip deletes all of them.
So structured edits rewrite the *text* in place -- same approach as
scripts/sync_initial_positions.py, generalised to arbitrary keys -- and the raw
editor writes exactly the bytes you typed after a yaml.safe_load() parse check.

Every write takes a timestamped backup under console/backups/ first.
"""

from __future__ import annotations

import datetime as _dt
import difflib
import math
import os
import re
import shutil

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
CFG_DIR = os.path.join(REPO, 'src', 'crazyswarm2', 'crazyflie', 'config')
BACKUP_DIR = os.path.join(REPO, 'console', 'backups')

FILES = {
    'crazyflies': {
        'path': os.path.join(CFG_DIR, 'crazyflies.yaml'),
        'label': 'crazyflies.yaml',
        'blurb': 'The fleet: which drones exist, their radio URI, where each one '
                 'physically sits, and the firmware logging/params pushed at connect. '
                 'Read ONLY at server launch.',
    },
    'motion_capture': {
        'path': os.path.join(CFG_DIR, 'motion_capture.yaml'),
        'label': 'motion_capture.yaml',
        'blurb': 'The OptiTrack link: parser type, Motive host ("auto" = NatNet '
                 'discovery ping), marker/dynamics configurations, /poses QoS.',
    },
    'server': {
        'path': os.path.join(CFG_DIR, 'server.yaml'),
        'label': 'server.yaml',
        'blurb': 'crazyflie_server behaviour: warning thresholds, firmware-param '
                 'query-on-connect, and the simulator backend settings.',
    },
    'teleop': {
        'path': os.path.join(CFG_DIR, 'teleop.yaml'),
        'label': 'teleop.yaml',
        'blurb': 'Gamepad mapping and limits for the built-in teleop node.',
    },
}

# Firmware log-variable widths, for the 26-byte-per-log-block budget check.
# Anything unknown is assumed to be a 4-byte float (the conservative guess).
VAR_BYTES = {
    'motion.deltaX': 2, 'motion.deltaY': 2, 'range.zrange': 2,
    'stateEstimateZ.vx': 2, 'stateEstimateZ.vy': 2, 'stateEstimateZ.vz': 2,
    'stateEstimateZ.x': 2, 'stateEstimateZ.y': 2, 'stateEstimateZ.z': 2,
    'supervisor.info': 2, 'pm.state': 1,
}
LOG_BLOCK_BYTES = 26
MIN_SEPARATION_M = 1.0
URI_RE = re.compile(r'^radio://([^/]+)/(\d+)/(\d+[MK])/([0-9A-Fa-f]+)(?:/.*)?$')


# --------------------------------------------------------------------------
# text-preserving editor
# --------------------------------------------------------------------------
class YamlText:
    """Minimal, indentation-driven text editor for well-formed block YAML.

    It only understands what these config files actually use: nested block
    mappings, scalars and inline flow lists. That is deliberate -- it has to be
    predictable, not general.
    """

    def __init__(self, path):
        self.path = path
        with open(path) as fh:
            self.lines = fh.readlines()
        self.original = ''.join(self.lines)

    # -- lookup ------------------------------------------------------------
    @staticmethod
    def _indent(line):
        return len(line) - len(line.lstrip())

    @staticmethod
    def _is_content(line):
        s = line.strip()
        return bool(s) and not s.startswith('#')

    def _child_indent(self, lo, hi):
        for i in range(lo, hi):
            if self._is_content(self.lines[i]):
                return self._indent(self.lines[i])
        return None

    def find(self, keys):
        """-> (line_index, indent, body_lo, body_hi) for the key path, or None."""
        lo, hi = 0, len(self.lines)
        result = None
        for key in keys:
            indent = self._child_indent(lo, hi)
            if indent is None:
                return None
            pattern = re.compile(r'^\s{%d}%s:\s*(.*)$' % (indent, re.escape(key)))
            hit = None
            for i in range(lo, hi):
                line = self.lines[i]
                if not self._is_content(line) or self._indent(line) != indent:
                    continue
                if pattern.match(line):
                    hit = i
                    break
            if hit is None:
                return None
            body_lo, body_hi = hit + 1, hi
            for j in range(hit + 1, hi):
                if self._is_content(self.lines[j]) and self._indent(self.lines[j]) <= indent:
                    body_hi = j
                    break
            result = (hit, indent, body_lo, body_hi)
            lo, hi = body_lo, body_hi
        return result

    def keys_under(self, keys):
        """Direct child keys of a mapping (commented-out lines are ignored)."""
        found = self.find(keys)
        if found is None:
            return []
        _, _, lo, hi = found
        indent = self._child_indent(lo, hi)
        if indent is None:
            return []
        out = []
        for i in range(lo, hi):
            line = self.lines[i]
            if self._is_content(line) and self._indent(line) == indent:
                m = re.match(r'^\s*([A-Za-z0-9_./-]+):', line)
                if m:
                    out.append(m.group(1))
        return out

    # -- mutation ----------------------------------------------------------
    def set_value(self, keys, raw_value):
        """Replace the scalar/flow value of a key, keeping any inline comment."""
        found = self.find(keys)
        if found is None:
            raise KeyError('/'.join(keys) + ': not found')
        i = found[0]
        m = re.match(r'^(\s*[^:]+:)([^#\n]*)(#.*)?$', self.lines[i])
        if not m:
            raise ValueError(f'{"/".join(keys)}: cannot edit line {i + 1} in place')
        if m.group(2).strip() == str(raw_value).strip():
            return              # unchanged: leave the line's own spacing alone
        comment = m.group(3) or ''
        pad = '  ' if comment else ''
        self.lines[i] = f'{m.group(1)} {raw_value}{pad}{comment}'.rstrip() + '\n'

    def remove_block(self, keys):
        found = self.find(keys)
        if found is None:
            raise KeyError('/'.join(keys) + ': not found')
        i, _, _, hi = found
        start = i
        # swallow the comment lines / blank line that belong to this block
        while start > 0 and self.lines[start - 1].strip().startswith('#'):
            start -= 1
        while hi < len(self.lines) and not self.lines[hi].strip():
            hi += 1
        del self.lines[start:hi]

    def append_child(self, keys, text_block):
        """Insert a block of text as the last child of `keys`."""
        found = self.find(keys)
        if found is None:
            raise KeyError('/'.join(keys) + ': not found')
        _, _, lo, hi = found
        at = hi
        while at > lo and not self.lines[at - 1].strip():
            at -= 1
        lines = [l + '\n' for l in text_block.rstrip('\n').split('\n')]
        self.lines[at:at] = ['\n'] + lines

    # -- output ------------------------------------------------------------
    def text(self):
        return ''.join(self.lines)

    def diff(self):
        return ''.join(difflib.unified_diff(
            self.original.splitlines(keepends=True), self.lines,
            fromfile=os.path.basename(self.path) + ' (current)',
            tofile=os.path.basename(self.path) + ' (new)'))


def fmt_float(v):
    """Keep it a YAML float: a bare `2` would load as int and make
    initial_position a mixed int/float array."""
    out = f'{round(float(v), 4):g}'
    return out + '.0' if ('.' not in out and 'e' not in out) else out


# --------------------------------------------------------------------------
# read / write
# --------------------------------------------------------------------------
def load(key):
    spec = FILES[key]
    with open(spec['path']) as fh:
        text = fh.read()
    try:
        doc = yaml.safe_load(text)
        error = None
    except yaml.YAMLError as exc:
        doc, error = None, str(exc)
    return {'key': key, 'path': spec['path'], 'label': spec['label'],
            'blurb': spec['blurb'], 'text': text, 'doc': doc, 'parse_error': error,
            'mtime': os.path.getmtime(spec['path'])}


def backup(path):
    os.makedirs(BACKUP_DIR, exist_ok=True)
    stamp = _dt.datetime.now().strftime('%Y%m%d-%H%M%S')
    dest = os.path.join(BACKUP_DIR, f'{os.path.basename(path)}.{stamp}')
    shutil.copy2(path, dest)
    return dest


def write(path, text, check_yaml=True):
    """Parse-check, back up, then write. -> (backup_path, diff)."""
    if check_yaml:
        yaml.safe_load(text)          # raises YAMLError -> reported to the UI
    with open(path) as fh:
        before = fh.read()
    if before == text:
        return None, ''
    bak = backup(path)
    tmp = path + '.console.tmp'
    with open(tmp, 'w') as fh:
        fh.write(text)
    os.replace(tmp, path)
    diff = ''.join(difflib.unified_diff(
        before.splitlines(keepends=True), text.splitlines(keepends=True),
        fromfile=os.path.basename(path) + ' (before)',
        tofile=os.path.basename(path) + ' (after)'))
    return bak, diff


ROBOT_TEMPLATE = """  {name}:
    enabled: {enabled}
    uri: {uri}
    initial_position: [{x}, {y}, {z}]
    type: {type}  # see robot_types"""


def apply_robot_edits(edits, adds=(), removes=()):
    """Apply structured crazyflies.yaml edits. -> (diff, text).

    edits: [{'name':'cf1','field':'enabled'|'uri'|'type'|'initial_position',
             'value': ...}]
    """
    ed = YamlText(FILES['crazyflies']['path'])
    for name in removes:
        ed.remove_block(['robots', name])
    for item in adds:
        block = ROBOT_TEMPLATE.format(
            name=item['name'],
            enabled='true' if item.get('enabled', True) else 'false',
            uri=item['uri'],
            x=fmt_float(item.get('x', 0.0)), y=fmt_float(item.get('y', 0.0)),
            z=fmt_float(item.get('z', 0.0)), type=item.get('type', 'cf21'))
        ed.append_child(['robots'], block)
    for e in edits:
        keys = ['robots', e['name'], e['field']]
        if e['field'] == 'initial_position':
            x, y, z = (fmt_float(v) for v in e['value'])
            ed.set_value(keys, f'[{x}, {y}, {z}]')
        elif e['field'] == 'enabled':
            ed.set_value(keys, 'true' if e['value'] else 'false')
        else:
            ed.set_value(keys, str(e['value']).strip())
    return ed.diff(), ed.text()


def apply_kv_edits(file_key, edits):
    """Generic 'set this key path to this scalar' edit for the other files."""
    ed = YamlText(FILES[file_key]['path'])
    for e in edits:
        ed.set_value(e['keys'], str(e['value']).strip())
    return ed.diff(), ed.text()


# --------------------------------------------------------------------------
# validation -- the checks that predict the failures this rig has actually hit
# --------------------------------------------------------------------------
def _finding(level, title, detail, fix=''):
    return {'level': level, 'title': title, 'detail': detail, 'fix': fix}


def validate_crazyflies(doc):
    out = []
    if not isinstance(doc, dict):
        return [_finding('fail', 'crazyflies.yaml did not parse', 'Top level is not a mapping.')]
    robots = doc.get('robots') or {}
    types = doc.get('robot_types') or {}
    enabled = {n: c for n, c in robots.items()
               if isinstance(c, dict) and c.get('enabled')}

    if not enabled:
        out.append(_finding('warn', 'No drone is enabled',
                            'Every robot has enabled: false, so the server will '
                            'connect to nothing.'))

    # --- radio URIs ---
    addresses, dongles = {}, {}
    for name, cfg in enabled.items():
        uri = str(cfg.get('uri', ''))
        m = URI_RE.match(uri)
        if not m:
            out.append(_finding('fail', f'{name}: URI not understood',
                                f'uri: {uri!r} does not look like '
                                'radio://<dongle>/<channel>/<datarate>/<address>.'))
            continue
        dongle, channel, datarate, addr = m.group(1), int(m.group(2)), m.group(3), m.group(4).upper()
        addresses.setdefault(addr, []).append(name)
        dongles.setdefault(dongle, set()).add((channel, datarate))
        tail = addr[-2:]
        digits = re.sub(r'\D', '', name)
        if digits and tail != f'{int(digits):02X}':
            out.append(_finding('info', f'{name}: address tail does not match the name',
                                f'{name} ends its address in 0x{tail}. Not an error, '
                                'but on this rig cfN normally uses ...E7{N:02X}.'))
    for addr, names in addresses.items():
        if len(names) > 1:
            out.append(_finding('fail', 'Duplicate radio address',
                                f'{", ".join(names)} all use address {addr}. '
                                'Each drone needs a unique address.'))
    real = [d for d in dongles if d != '*']
    if len(real) > 1:
        chans = sorted({c for d in real for c, _ in dongles[d]})
        if any(abs(a - b) < 2 for a in chans for b in chans if a != b):
            out.append(_finding('fail', 'Two dongles, channels less than 2 apart',
                                f'Channels in use: {chans}. A 2M channel is ~2 MHz wide, so '
                                'crazyflie-link-cpp refuses the pair ("Channels 80 and 81 are '
                                'already served by Crazyradio 0").',
                                'Space the channels >= 2 apart, e.g. 80 and 90.'))

    # --- geometry ---
    positions = {}
    for name, cfg in enabled.items():
        pos = cfg.get('initial_position')
        if not (isinstance(pos, (list, tuple)) and len(pos) == 3):
            out.append(_finding('fail', f'{name}: initial_position is not [x, y, z]',
                                f'Got {pos!r}.'))
            continue
        positions[name] = [float(v) for v in pos]
        if abs(positions[name][2]) > 0.5:
            out.append(_finding('warn', f'{name}: initial_position z = {positions[name][2]:.2f} m',
                                'A drone sitting on the floor should have z near 0.'))
    names = sorted(positions)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            d = math.dist(positions[a][:2], positions[b][:2])
            if d < MIN_SEPARATION_M:
                out.append(_finding(
                    'fail', f'{a} and {b} start {d:.2f} m apart',
                    f'The rig rule is >= {MIN_SEPARATION_M:.1f} m between enabled drones: '
                    'the trajectory demos preserve start separation, so a tight start '
                    'stays tight in the air.',
                    'Move the drones, then re-run "Sync initial positions from /poses".'))

    # --- types ---
    for name, cfg in enabled.items():
        t = cfg.get('type')
        if t not in types:
            out.append(_finding('fail', f'{name}: unknown type {t!r}',
                                f'robot_types defines: {", ".join(sorted(types))}.'))

    # --- firmware log-block budget ---
    for scope, block in _logging_blocks(doc):
        for topic, spec in (block.get('custom_topics') or {}).items():
            variables = (spec or {}).get('vars') or []
            size = sum(VAR_BYTES.get(v, 4) for v in variables)
            if size > LOG_BLOCK_BYTES:
                out.append(_finding(
                    'fail', f'Log block {topic} is {size} B (limit {LOG_BLOCK_BYTES} B)',
                    f'{scope}.firmware_logging.custom_topics.{topic} lists '
                    f'{len(variables)} vars. The firmware rejects an oversized block.',
                    'Drop a variable or split the topic in two.'))
            elif size > LOG_BLOCK_BYTES - 4:
                out.append(_finding(
                    'info', f'Log block {topic} is {size} B of {LOG_BLOCK_BYTES} B',
                    'Close to the per-block limit; one more float will not fit.'))
    return out


def _logging_blocks(doc):
    if isinstance(doc.get('all'), dict) and doc['all'].get('firmware_logging'):
        yield 'all', doc['all']['firmware_logging']
    for name, cfg in (doc.get('robot_types') or {}).items():
        if isinstance(cfg, dict) and cfg.get('firmware_logging'):
            yield f'robot_types.{name}', cfg['firmware_logging']
    for name, cfg in (doc.get('robots') or {}).items():
        if isinstance(cfg, dict) and cfg.get('firmware_logging'):
            yield f'robots.{name}', cfg['firmware_logging']


def validate_motion_capture(doc):
    out = []
    params = ((doc or {}).get('/motion_capture_tracking') or {}).get('ros__parameters') or {}
    host = str(params.get('hostname', '')).strip()
    if host.lower() != 'auto':
        if not re.match(r'^\d+\.\d+\.\d+\.\d+$', host):
            out.append(_finding('fail', 'Motive hostname is not an IPv4 literal',
                                f'hostname: {host!r}. The NatNet parser does not resolve names.',
                                'Use "auto" (NatNet discovery ping) or a literal like 192.168.9.152.'))
        else:
            out.append(_finding('info', f'Motive address pinned to {host}',
                                'The lab Motive PC uses DHCP and its address has drifted '
                                '(.100 -> .124 -> .152); each drift silently killed /poses.',
                                'hostname: "auto" lets launch.py discover it each time.'))
    if params.get('type') == 'optitrack_closed_source':
        out.append(_finding('warn', 'Closed-source OptiTrack parser selected',
                            'It wedges permanently after a Wi-Fi multicast stall.',
                            'Use type: "optitrack".'))
    deadline = (((params.get('topics') or {}).get('poses') or {}).get('qos') or {}).get('deadline')
    if deadline and abs(float(deadline) - 50.0) > 1e-6:
        out.append(_finding('info', f'/poses QoS deadline is {deadline} Hz',
                            'It should match Motive\'s streaming rate (50 Hz on this rig).'))
    return out


def fleet_summary(doc):
    """Compact fleet view for the health graph and the header."""
    robots = (doc or {}).get('robots') or {}
    out = []
    for name, cfg in sorted(robots.items()):
        if not isinstance(cfg, dict):
            continue
        uri = str(cfg.get('uri', ''))
        m = URI_RE.match(uri)
        out.append({
            'name': name,
            'enabled': bool(cfg.get('enabled')),
            'uri': uri,
            'type': cfg.get('type'),
            'address': ('0x' + m.group(4).upper()) if m else None,
            'channel': int(m.group(2)) if m else None,
            'datarate': m.group(3) if m else None,
            'initial_position': cfg.get('initial_position'),
        })
    return out
