"""crazyflie_follow -- ROS 2 adapter around follow_core for the Crazyflie
prototyping phase of the person-following drone project.

This package is the ADAPTER, not the algorithm. The control law, arbiter,
speed governor and track supervisor all live in follow_core
(~/near-intern/follow-drone/follow_core), a platform-independent package with
no ROS/cflib dependency, developed and unit-tested on its own so that the
exact same code which gets validated here against OptiTrack later flies on
the Starling against real UWB + vision. Nothing in this package reimplements,
copies, or patches that logic -- see HANDOVER.md "What this package is".

follow_core has no setup.py/pyproject.toml of its own, by design: it is
imported by inserting its repo root onto sys.path, the same way
follow-drone/tests/conftest.py already does for the unit tests. That is fine
inside follow-drone/, but this package is built to be copied out of
~/near-intern/follow-drone/crazyflie/ into ~/CrazySwarm2-Strike-and-Guard/src/ as a
drop-in (see HANDOVER.md) -- and follow_core's repo root will *not*
automatically be on sys.path in that workspace's Python environment.

So this shim resolves it, in priority order, the first time anything in this
package is imported:

  1. follow_core is already importable (e.g. PYTHONPATH set in the shell
     that sourced the workspace) -- do nothing.
  2. $FOLLOW_CORE_PATH, if set, names the follow-drone repo root explicitly
     and is prepended to sys.path.
  3. This file's own location on disk, walked up three directories. While
     this package still lives at its native path
     (.../follow-drone/crazyflie/crazyflie_follow/crazyflie_follow/__init__.py)
     that lands on .../follow-drone, which is exactly where follow_core is.
     Once the package is copied elsewhere this guess naturally stops
     resolving (there is nothing to walk up to any more), which is why it is
     tried before, not instead of, option 4.
  4. ~/near-intern/follow-drone, the fixed path this whole project has lived
     at throughout development -- the common case when this package has been
     copied into ~/CrazySwarm2-Strike-and-Guard/src/ but follow-drone itself has
     not moved.

If none of those make `import follow_core` succeed, every entry point in this
package fails immediately, at import time, with the exact fix -- rather than
failing deep inside a control loop after the drone is already armed.
"""
import os
import sys


def _ensure_follow_core_importable() -> None:
    try:
        import follow_core  # noqa: F401
        return
    except ImportError:
        pass

    candidates = []
    env = os.environ.get('FOLLOW_CORE_PATH')
    if env:
        candidates.append(env)

    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.abspath(os.path.join(here, '..', '..', '..')))
    candidates.append(os.path.expanduser('~/near-intern/follow-drone'))

    for c in candidates:
        if c and os.path.isdir(os.path.join(c, 'follow_core')) and c not in sys.path:
            sys.path.insert(0, c)
            try:
                import follow_core  # noqa: F401
                return
            except ImportError:
                continue

    raise ImportError(
        "crazyflie_follow requires follow_core, which this package "
        "deliberately does not vendor a copy of (see HANDOVER.md). Make it "
        "importable with one of:\n"
        "    export PYTHONPATH=/path/to/follow-drone:$PYTHONPATH\n"
        "    export FOLLOW_CORE_PATH=/path/to/follow-drone\n"
        "before sourcing the workspace / launching any crazyflie_follow node "
        "or script."
    )


_ensure_follow_core_importable()
