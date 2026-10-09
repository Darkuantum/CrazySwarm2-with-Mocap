"""Usage log: what the operator actually uses, kept across sessions.

The dashboard's layout used to be a guess about which buttons matter. This is
the record that replaces the guess: one JSON line per thing the operator did,
appended to ``console/usage/usage.jsonl`` (git-ignored -- it is this laptop's
history, not repo content). ``summary()`` folds it into counts the dashboard
uses to order its overflow menu and to suggest shortcuts, and the Command log
tab shows it so a human can read it too. A Claude session on this machine can
simply ``cat`` the file.

What is recorded, and what is NOT:

* actions run (catalog id, where they were started from, the parameter
  values), e-stops, stops, config writes, scans, mission opens/starts, tab and
  palette use;
* for stdin only the SHAPE of the line -- ``enter``, ``q`` or ``text`` --
  never its content, because the stdin box is also where a sudo password goes;
* teleop keystrokes are not logged one by one (ten a second would drown
  everything else); a teleop session is logged once when it starts.

Every write is best-effort: a full disk or a read-only checkout must cost the
log, never an action.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import Counter, defaultdict

# MISSION_CONSOLE_USAGE_DIR redirects it -- for a test console, whose clicks
# must not be mistaken for the operator's.
DIR = (os.environ.get('MISSION_CONSOLE_USAGE_DIR') or
       os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'usage')))
LOG = os.path.join(DIR, 'usage.jsonl')
PREFS = os.path.join(DIR, 'dashboard.json')

#: Events the browser may report through /api/usage. Anything else is dropped,
#: so a page cannot fill the log with arbitrary junk.
CLIENT_EVENTS = {'tab', 'palette', 'nav', 'more', 'step', 'mission_open', 'teleop'}

_lock = threading.Lock()
SESSION = time.strftime('%Y%m%d-%H%M%S')


def record(ev, **fields):
    """Append one event. Never raises."""
    row = {'ts': round(time.time(), 3), 'session': SESSION, 'ev': ev}
    row.update({k: v for k, v in fields.items() if v is not None})
    try:
        line = json.dumps(row, default=str, separators=(',', ':'))
        with _lock:
            os.makedirs(DIR, exist_ok=True)
            with open(LOG, 'a', encoding='utf-8') as fh:
                fh.write(line + '\n')
    except Exception:                                     # noqa: BLE001
        pass


def stdin_shape(text):
    """'enter' | 'q' | 'text' -- the content itself is never stored."""
    t = (text or '').strip()
    if not t:
        return 'enter'
    if t in ('q', 'Q'):
        return 'q'
    return 'text'


def _rows(since=None):
    try:
        with open(LOG, encoding='utf-8') as fh:
            for raw in fh:
                try:
                    row = json.loads(raw)
                except ValueError:
                    continue
                if since is None or row.get('ts', 0) >= since:
                    yield row
    except OSError:
        return


def summary(days=None, labels=None):
    """Fold the log into what the UI needs.

    days:   only count the last N days (None = everything).
    labels: {action_id: label}, so the summary reads in words.
    """
    labels = labels or {}
    since = time.time() - days * 86400 if days else None
    actions = defaultdict(lambda: {'count': 0, 'last': 0.0, 'sources': Counter()})
    tabs, sources, events = Counter(), Counter(), Counter()
    missions = defaultdict(lambda: {'opens': 0, 'starts': 0, 'last': 0.0})
    sessions, first = set(), None
    for r in _rows(since):
        ev, ts = r.get('ev'), r.get('ts', 0.0)
        first = ts if first is None else min(first, ts)
        sessions.add(r.get('session'))
        events[ev] += 1
        if ev == 'run':
            a = actions[r.get('action')]
            a['count'] += 1
            a['last'] = max(a['last'], ts)
            a['sources'][r.get('source') or 'unknown'] += 1
            sources[r.get('source') or 'unknown'] += 1
        elif ev == 'tab':
            tabs[r.get('tab')] += 1
        elif ev == 'mission_open' or (ev == 'mission_start' and r.get('role') == 'main'):
            # a helper (the teleop) starting is part of a run, not another run
            m = missions[r.get('mission')]
            m['opens' if ev == 'mission_open' else 'starts'] += 1
            m['last'] = max(m['last'], ts)
    return {
        'log': LOG,
        'since': first,
        'sessions': len(sessions),
        'events': dict(events),
        'actions': sorted(({'id': k, 'label': labels.get(k, k), 'count': v['count'],
                            'last': v['last'], 'sources': dict(v['sources'])}
                           for k, v in actions.items() if k),
                          key=lambda a: (-a['count'], -a['last'])),
        'tabs': dict(tabs.most_common()),
        'sources': dict(sources.most_common()),
        'missions': {k: v for k, v in missions.items() if k},
    }


# ---------------------------------------------------------------- prefs
# The operator's own dashboard choices (pinned shortcuts). Kept next to the
# log rather than in browser storage so they survive a different browser or
# profile, and so they are readable from a shell.

def load_prefs():
    try:
        with open(PREFS, encoding='utf-8') as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_prefs(prefs):
    prefs = {'pins': [str(p) for p in (prefs.get('pins') or [])][:24]}
    try:
        with _lock:
            os.makedirs(DIR, exist_ok=True)
            tmp = PREFS + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(prefs, fh, indent=1)
            os.replace(tmp, PREFS)
    except OSError:
        pass
    return prefs
