"""Mission views: a dashboard per mission, defined BY the mission, not by the console.

The console is generic on purpose; a demo is not. The escort needs a gate you
answer and a VIP you steer, the next demo will need something else, and
editing the console for each one is how it turns into a pile of special
cases. So a show package ships its own mission spec next to its code::

    <package source>/missions/<name>.yaml      (e.g. crazyflie_shows/missions/escort.yaml)

and the console discovers it exactly the way it discovers flight scripts --
every package that depends on ``crazyflie_py`` is scanned. The spec says what
to run, which choices the operator makes before starting, which helper
processes (keyboard teleop, a viz node) go with it, which topics to draw, how
to recognise a prompt that waits for the operator, and what to pull out of the
script's output as status. The mission window (``/mission?m=<pkg>/<name>``)
renders all of that; a spec can also ship its own view module for anything the
stock widgets cannot draw. Both live with the show, so a new demo is a new
YAML file, not a console change.

Specs are read from the package's SOURCE directory when the installed
``package.xml`` symlinks back to it (``colcon build --symlink-install``, which
``scripts/build.sh`` uses), so editing a spec needs no rebuild; otherwise from
``install/<pkg>/share/<pkg>/missions``.

Same rule as the catalog: the argv built here is the argv executed and the
argv the window shows. Operator choices are validated against the spec --
a select must be one of its choices, a number must parse -- because
``ros2 run`` silently ignores a misspelled parameter and flies the default.
"""

from __future__ import annotations

import glob
import os
import re
import shlex

import yaml

from . import catalog

INSTALL = catalog.INSTALL
SPEC_VERSION = 1
NAME_RE = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_.-]*$')

#: Key maps for the teleop widget. Letters only: they are written to the
#: helper's pty, and escort_teleop reads raw keys (terminal auto-repeat is
#: what keeps a held key moving, so the widget re-sends while held).
TELEOP_PRESETS = {
    'planar': [
        {'label': '+y', 'key': 'w', 'kbd': 'KeyW', 'cell': [1, 0], 'hold': True},
        {'label': '-x', 'key': 'a', 'kbd': 'KeyA', 'cell': [0, 1], 'hold': True},
        {'label': 'stop', 'key': ' ', 'kbd': 'Space', 'cell': [1, 1], 'hold': False},
        {'label': '+x', 'key': 'd', 'kbd': 'KeyD', 'cell': [2, 1], 'hold': True},
        {'label': '-y', 'key': 's', 'kbd': 'KeyS', 'cell': [1, 2], 'hold': True},
        {'label': 'centre', 'key': 'c', 'kbd': 'KeyC', 'cell': [3, 2], 'hold': False},
    ],
}
TELEOP_PRESETS['spatial'] = TELEOP_PRESETS['planar'] + [
    {'label': 'up', 'key': 'q', 'kbd': 'KeyQ', 'cell': [3, 0], 'hold': True},
    {'label': 'down', 'key': 'e', 'kbd': 'KeyE', 'cell': [3, 1], 'hold': True},
]

#: A prompt line most scripts here use (escort_show's operator_gate prints
#: ">>> ..."), and the ending of an input() prompt that has no newline yet.
DEFAULT_GATE = {
    'prompt': r'^\s*>>>\s*(?P<text>.+)$',
    'until': r'\[q\]\s*land|continue\b',
    'partial': r'(\?|:|>)\s*$|\b[Ee]nter\b',
    'buttons': [
        {'label': 'Continue', 'send': '', 'key': 'Enter', 'tone': 'primary'},
        {'label': 'Land (q)', 'send': 'q', 'tone': 'danger'},
    ],
}
DEFAULT_LAYOUT = {
    'main': ['scene'],
    'side': ['gate', 'indicators', 'teleop', 'fleet', 'launch'],
    'bottom': ['events', 'output'],
}
WIDGETS = {'scene', 'gate', 'indicators', 'teleop', 'fleet', 'launch', 'events', 'output'}


class SpecError(ValueError):
    pass


# ------------------------------------------------------------- discovery
def _mission_dirs():
    """[(pkg, missions_dir)] for every crazyflie_py-dependent package."""
    out = []
    for xml in sorted(glob.glob(os.path.join(INSTALL, '*', 'share', '*', 'package.xml'))):
        pkg = os.path.basename(os.path.dirname(xml))
        try:
            with open(xml, encoding='utf-8') as fh:
                body = fh.read()
        except OSError:
            continue
        if not re.search(r'<(?:exec_|build_)?depend>\s*crazyflie_py\s*<', body):
            continue
        src = os.path.dirname(os.path.realpath(xml))
        for cand in (os.path.join(src, 'missions'),
                     os.path.join(os.path.dirname(xml), 'missions')):
            if os.path.isdir(cand):
                out.append((pkg, cand))
                break
    return out


def discover():
    """Every mission spec, parsed. A broken spec is listed with its error
    rather than dropped -- a demo that silently vanished from the list is
    harder to fix than one that says why it cannot load."""
    found = [load_builtin()]
    for pkg, d in _mission_dirs():
        for path in sorted(glob.glob(os.path.join(d, '*.yaml'))):
            stem = os.path.splitext(os.path.basename(path))[0]
            mid = f'{pkg}/{stem}'
            try:
                found.append(load(path, pkg, mid))
            except Exception as exc:                     # noqa: BLE001
                found.append({'id': mid, 'pkg': pkg, 'title': stem, 'error': str(exc),
                              'path': path, 'summary': ''})
    return found


def get(mid):
    for m in discover():
        if m['id'] == mid:
            if m.get('error'):
                raise SpecError(f'{mid}: {m["error"]}')
            return m
    raise KeyError(mid)


def signature():
    """Change detector for the cache: every spec file and its mtime."""
    sig = []
    for _, d in _mission_dirs():
        for path in sorted(glob.glob(os.path.join(d, '*'))):
            try:
                sig.append((path, os.path.getmtime(path)))
            except OSError:
                pass
    return tuple(sig)


# ---------------------------------------------------------------- parsing
def load(path, pkg, mid):
    with open(path, encoding='utf-8') as fh:
        raw = yaml.safe_load(fh) or {}
    if not isinstance(raw, dict):
        raise SpecError('the file is not a mapping')
    return normalise(raw, pkg, mid, os.path.dirname(path))


def _str(v, what):
    if not isinstance(v, str) or not v.strip():
        raise SpecError(f'{what} must be a non-empty string')
    return v.strip()


_PY_GROUP = re.compile(r'\(\?P<')
_JS_GROUP = re.compile(r'\(\?<(?![=!])')
_INLINE_FLAGS = re.compile(r'\(\?[aiLmsux-]+[:)]')


def _regex(v, what):
    """Validate a spec regex and return it in the BROWSER's dialect.

    The patterns run in the mission window (JavaScript), but a spec author
    will write Python's ``(?P<name>...)``, which JavaScript rejects outright
    -- and JavaScript's ``(?<name>...)`` is a syntax error in Python 3.10. So
    either spelling is accepted, checked here as Python, and shipped as
    JavaScript. A LEADING ``(?i)`` is allowed too (the window turns it into
    the ``i`` flag); any other inline flag is refused, since JavaScript has
    none. Everything else in the two dialects agrees for the patterns a spec
    needs (classes, anchors, alternation, quantifiers).
    """
    if _INLINE_FLAGS.search(v[4:] if v.startswith('(?i)') else v):
        raise SpecError(f'{what}: {v!r} -- the only inline flag both dialects can '
                        'share is a leading (?i)')
    try:
        re.compile(_JS_GROUP.sub('(?P<', v))
    except re.error as exc:
        raise SpecError(f'{what}: bad regex {v!r} ({exc})') from exc
    return _PY_GROUP.sub('(?<', v)


def _run_target(pkg, run, what):
    """'exe' (same package) or 'pkg exe' -> (pkg, exe)."""
    parts = _str(run, what).split()
    if len(parts) == 1:
        return pkg, parts[0]
    if len(parts) == 2:
        return parts[0], parts[1]
    raise SpecError(f'{what}: expected "executable" or "package executable", got {run!r}')


def _option(o, i):
    if not isinstance(o, dict):
        raise SpecError(f'options[{i}] must be a mapping')
    name = _str(o.get('name'), f'options[{i}].name')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.]*', name):
        raise SpecError(f'options[{i}].name {name!r} is not a parameter name')
    choices = o.get('choices')
    typ = o.get('type') or ('select' if choices else 'text')
    if typ not in ('select', 'bool', 'number', 'text'):
        raise SpecError(f'option {name}: type must be select|bool|number|text')
    opts, descs = [], {}
    if typ == 'select':
        if isinstance(choices, dict):
            for k, v in choices.items():
                opts.append(str(k))
                descs[str(k)] = str(v or '')
        elif isinstance(choices, list) and choices:
            opts = [str(c) for c in choices]
        else:
            raise SpecError(f'option {name}: a select needs choices')
    default = o.get('default')
    if typ == 'bool':
        default = bool(default)
    elif typ == 'select':
        default = str(default) if default is not None else opts[0]
        if default not in opts:
            raise SpecError(f'option {name}: default {default!r} is not one of its choices')
    elif default is not None:
        default = str(default)
    when = {str(k): (str(v).lower() if isinstance(v, bool) else str(v))
            for k, v in (o.get('when') or {}).items()}
    return {'name': name, 'param': o.get('param') or name, 'label': o.get('label') or name,
            'type': typ, 'choices': opts, 'descriptions': descs, 'default': default,
            'help': str(o.get('help') or ''), 'when': when,
            # omit from the command line while it still equals its default
            'omit_default': bool(o.get('omit_default', False))}


def _teleop(v, what):
    if not v:
        return None
    if isinstance(v, str):
        if v not in TELEOP_PRESETS:
            raise SpecError(f'{what}: unknown preset {v!r} ({", ".join(TELEOP_PRESETS)})')
        return [dict(k) for k in TELEOP_PRESETS[v]]
    if isinstance(v, list):
        keys = []
        for i, k in enumerate(v):
            if not isinstance(k, dict) or not isinstance(k.get('key'), str) or len(k['key']) != 1:
                raise SpecError(f'{what}[{i}] needs a one-character key')
            keys.append({'label': str(k.get('label') or k['key']), 'key': k['key'],
                         'kbd': k.get('kbd'), 'cell': k.get('cell'),
                         'hold': bool(k.get('hold', False))})
        return keys
    raise SpecError(f'{what}: a preset name or a list of keys')


def normalise(raw, pkg, mid, base_dir):
    ver = raw.get('mission', SPEC_VERSION)
    if ver != SPEC_VERSION:
        raise SpecError(f'spec version {ver!r}; this console reads version {SPEC_VERSION}')
    title = _str(raw.get('title'), 'title')
    run = raw.get('run')
    if run == '{script}':
        main = ('', '')                       # chosen by the operator (built-in mission)
    else:
        main = _run_target(pkg, run, 'run')
    options = [_option(o, i) for i, o in enumerate(raw.get('options') or [])]
    names = [o['name'] for o in options]
    if len(set(names)) != len(names):
        raise SpecError('two options share a name')
    for o in options:
        for k in o['when']:
            if k not in names or k == o['name']:
                raise SpecError(f'option {o["name"]}: when names {k!r}, which is not another option')

    helpers = {}
    for key, h in (raw.get('helpers') or {}).items():
        if not NAME_RE.match(str(key)) or key == 'main':
            raise SpecError(f'helper name {key!r} is not usable')
        if not isinstance(h, dict):
            raise SpecError(f'helper {key} must be a mapping')
        hp, he = _run_target(pkg, h.get('run'), f'helpers.{key}.run')
        params = {}
        for k, v in (h.get('params') or {}).items():
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.]*', str(k)):
                raise SpecError(f'helpers.{key}.params: {k!r} is not a parameter name')
            params[str(k)] = str(v).lower() if isinstance(v, bool) else str(v)
        when = {str(k): (str(v).lower() if isinstance(v, bool) else str(v))
                for k, v in (h.get('when') or {}).items()}
        for k in when:
            if k not in names:
                raise SpecError(f'helpers.{key}.when names {k!r}, which is not an option')
        helpers[key] = {
            'key': key, 'label': str(h.get('label') or key), 'pkg': hp, 'exe': he,
            'params': params, 'when': when, 'autostart': bool(h.get('autostart', False)),
            'teleop': _teleop(h.get('teleop'), f'helpers.{key}.teleop'),
            # a regex on the MAIN script's output; the teleop pad stays
            # locked until it has been seen (e.g. 'ESCORT LIVE')
            'enable_after': (_regex(str(h['enable_after']), f'helpers.{key}.enable_after')
                             if h.get('enable_after') else None),
            'note': str(h.get('note') or ''),
        }

    gate = dict(DEFAULT_GATE)
    g = raw.get('gate')
    if g is False:
        gate = None
    elif isinstance(g, dict):
        gate.update({k: v for k, v in g.items() if k in ('prompt', 'until', 'partial', 'buttons')})
    if gate:
        for k in ('prompt', 'until', 'partial'):
            if gate.get(k):
                gate[k] = _regex(gate[k], f'gate.{k}')
        for i, b in enumerate(gate.get('buttons') or []):
            if not isinstance(b, dict) or 'send' not in b:
                raise SpecError(f'gate.buttons[{i}] needs a label and a `send` line')

    indicators = []
    for i, ind in enumerate(raw.get('indicators') or []):
        if not isinstance(ind, dict):
            raise SpecError(f'indicators[{i}] must be a mapping')
        indicators.append({
            'label': _str(ind.get('label'), f'indicators[{i}].label'),
            'match': _regex(_str(ind.get('match'), f'indicators[{i}].match'),
                            f'indicators[{i}].match'),
            'value': str(ind.get('value', '$1')),
            'tone': {str(k): str(v) for k, v in (ind.get('tone') or {}).items()},
            'initial': str(ind.get('initial', '--')),
        })
    events = [_regex(str(e), 'events') for e in (raw.get('events') or [])]

    topics = raw.get('topics') or {}
    markers = topics.get('markers') or []
    if isinstance(markers, str):
        markers = [markers]

    layout = dict(DEFAULT_LAYOUT)
    if isinstance(raw.get('layout'), dict):
        for slot in ('main', 'side', 'bottom'):
            if slot in raw['layout']:
                layout[slot] = [str(w) for w in (raw['layout'][slot] or [])]
    view = raw.get('view')
    if view is not None:
        view = _str(view, 'view')
        if '/' in view or not view.endswith(('.js', '.mjs')):
            raise SpecError('view must be a .js/.mjs file name next to the spec')
        if not os.path.isfile(os.path.join(base_dir, view)):
            raise SpecError(f'view {view} does not exist next to the spec')
    for slot, ws in layout.items():
        for w in ws:
            if w.split(':')[0] not in WIDGETS and not view:
                raise SpecError(f'layout.{slot}: unknown widget {w!r} '
                                f'(stock: {", ".join(sorted(WIDGETS))}; '
                                'custom widgets need a view module)')

    abort = str(raw.get('abort') or 'sigint')
    if abort not in ('sigint', 'q'):
        raise SpecError('abort must be sigint (the shows\' abort.py lands on it) or q')

    return {
        'id': mid, 'pkg': pkg, 'title': title,
        'summary': ' '.join(str(raw.get('summary') or '').split()),
        'docs': str(raw.get('docs') or ''),
        'run': {'pkg': main[0], 'exe': main[1]},
        'flight': bool(raw.get('flight', True)),
        'abort': abort,
        'options': options, 'helpers': helpers, 'gate': gate,
        'indicators': indicators, 'events': events,
        'topics': {'markers': [str(t) for t in markers]},
        'layout': layout, 'view': view, 'base_dir': base_dir,
        'path': os.path.join(base_dir, ''), 'error': None,
    }


def load_builtin():
    """'Any flight script': the generic mission view, so every script --
    including ones nobody wrote a spec for -- gets the 3D view, the gate
    buttons and the abort, with no file anywhere."""
    flights = [d for d in catalog.discover_scripts() if not d['ground']]
    choices = {f"{d['pkg']} {d['name']}": d['doc'] for d in flights}
    raw = {
        'title': 'Any flight script',
        'summary': 'The generic mission view for any script in the workspace: 3D '
                   'scene, prompt buttons, abort. Write a missions/<name>.yaml in '
                   'the show package to give a mission its own controls.',
        'run': '{script}',
        'options': ([{'name': 'script', 'label': 'script', 'choices': choices,
                      'default': next((k for k in choices if k.endswith(' hello_world')),
                                      next(iter(choices), None))}]
                    if choices else []),
        'layout': {'main': ['scene'], 'side': ['gate', 'fleet', 'launch'],
                   'bottom': ['events', 'output']},
        'events': [r'\*\*\*', r'\[t\+', r'ABORT', r'(?i)\b(error|refus|fail)'],
    }
    m = normalise(raw, 'console', 'console/script', os.path.dirname(__file__))
    m['builtin'] = True
    return m


def public(m):
    """The spec without filesystem paths the browser has no use for."""
    return {k: v for k, v in m.items() if k not in ('base_dir', 'path')}


# ------------------------------------------------------------------- argv
def _value(opt, values):
    v = values.get(opt['name'], opt['default'])
    if v is None or v == '':
        return None
    if opt['type'] == 'bool':
        if isinstance(v, str):
            v = v.strip().lower() in ('1', 'true', 'yes', 'on')
        return 'true' if v else 'false'
    v = str(v).strip()
    if opt['type'] == 'select' and v not in opt['choices']:
        raise ValueError(f'{opt["label"]}: {v!r} is not one of {", ".join(opt["choices"])}')
    if opt['type'] == 'number':
        try:
            float(v)
        except ValueError:
            raise ValueError(f'{opt["label"]}: {v!r} is not a number') from None
    if opt['type'] == 'text' and not re.fullmatch(r'[^\s]+', v):
        raise ValueError(f'{opt["label"]}: no spaces allowed ({v!r})')
    return v


def _applies(m, when, values):
    for k, want in when.items():
        opt = next(o for o in m['options'] if o['name'] == k)
        if _value(opt, values) != want:
            return False
    return True


def active_helpers(m, values):
    """Helpers whose `when` matches the operator's choices."""
    return [h for h in m['helpers'].values() if _applies(m, h['when'], values)]


def build(m, role, values, sim=False):
    """-> (argv, cmdline, label) for the main script or one helper."""
    values = values or {}
    params = ['use_sim_time:=true'] if sim else []
    if role == 'main':
        if m['run']['exe']:
            pkg, exe = m['run']['pkg'], m['run']['exe']
        else:
            pkg, _, exe = str(values.get('script') or '').partition(' ')
            allowed = next((o['choices'] for o in m['options'] if o['name'] == 'script'), [])
            if f'{pkg} {exe}' not in allowed:
                raise ValueError('pick a script from the list')
        for opt in m['options']:
            if m['run']['exe'] == '' and opt['name'] == 'script':
                continue
            if not _applies(m, opt['when'], values):
                continue
            v = _value(opt, values)
            if v is None:
                continue
            if opt['omit_default'] and v == _value(opt, {}):
                continue
            params.append(f'{opt["param"]}:={v}')
        params += catalog._ros_params(values.get('extra', ''))
        label = m['title']
    else:
        h = m['helpers'].get(role)
        if h is None:
            raise ValueError(f'no helper called {role!r} in {m["id"]}')
        pkg, exe = h['pkg'], h['exe']
        params += [f'{k}:={v}' for k, v in h['params'].items()]
        label = f'{m["title"]} - {h["label"]}'
    argv = ['ros2', 'run', pkg, exe]
    if params:
        argv.append('--ros-args')
        for kv in params:
            argv += ['-p', kv]
    return argv, ' '.join(shlex.quote(a) for a in argv), label


def action_id(mid, role):
    return f'mission:{mid}:{role}'


def asset_path(m, rel):
    """A file next to the spec (a custom view module), path-checked."""
    base = os.path.realpath(m['base_dir'])
    full = os.path.realpath(os.path.join(base, rel))
    if not full.startswith(base + os.sep) or not os.path.isfile(full):
        return None
    return full
