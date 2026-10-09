"""One store for every choice the operator makes -- the single source of truth.

Every dropdown and field used to remember its value per button, in the
browser: the Control tab's `rviz` was not the Dashboard's `rviz`, "Restart"
had its own `backend`, and the mission window had its own copy of everything.
Choose `rviz: False` on one, launch from another, get RViz.

Now a choice is stored ONCE, here, server-side (console/usage/settings.json,
next to the usage log), under a key that says what it IS rather than which
button showed it:

* launch arguments (`backend`, `rviz`, `preflight`, `foxglove`, `teleop`,
  `mocap`, `gui`, `debug`, `mocap_hostname`) and `drone` are one setting each,
  whichever card or button shows them -- see catalog.SHARED_PARAMS;
* everything else is keyed `<action>.<param>` (a takeoff height is not a
  landing height) or `mission.<id>.<option>`;

and every page reads it, writes it, and is told when it changes (an SSE
`settings` event). catalog.render() merges it under whatever the request
carried, so a command started from a pin, the palette or a mission window
uses the same values as the one started from its card.
"""

from __future__ import annotations

import json
import os
import re
import threading

from . import usage

_KEY = re.compile(r'^[A-Za-z0-9_.:/-]{1,160}$')
_lock = threading.Lock()


def path():
    return os.path.join(usage.DIR, 'settings.json')


def load():
    try:
        with open(path(), encoding='utf-8') as fh:
            data = json.load(fh)
        return {str(k): str(v) for k, v in data.items()} if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def update(values):
    """Merge {key: value} (value None = forget the key) and persist."""
    with _lock:
        cur = load()
        for k, v in (values or {}).items():
            if not _KEY.match(str(k)):
                raise ValueError(f'bad settings key {k!r}')
            if v is None:
                cur.pop(k, None)
            else:
                v = str(v)
                if len(v) > 400:
                    raise ValueError(f'{k}: value too long')
                cur[k] = v
        try:
            os.makedirs(os.path.dirname(path()), exist_ok=True)
            tmp = path() + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(cur, fh, indent=1, sort_keys=True)
            os.replace(tmp, path())
        except OSError:
            pass
        return cur
