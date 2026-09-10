"""Make the platform-independent `follow_core` package importable.

follow_core (owned by another team in this repo, at repo_root/follow_core/)
is deliberately NOT a ROS package and has no setup.py/colcon build of its own
-- it is meant to be shared byte-for-byte between the Crazyflie testbed and
this ROS 2 stack. That means it is not something `colcon build` installs for
us; it has to already be on sys.path when a follow_ros2 node starts.

Resolution order:
  1. It's already importable (e.g. you `pip install -e` it, or set
     PYTHONPATH yourself). Nothing to do -- this is the recommended setup for
     a real VOXL 2 deployment; see README.md.
  2. FOLLOW_CORE_PATH environment variable, if set, names the directory that
     *contains* follow_core/ (i.e. the repo root, not follow_core/ itself).
  3. Dev-workstation fallback: walk up from this file's real path looking for
     a sibling follow_core/ directory. This only works if colcon was built
     with `--symlink-install` (the README recommends this for local dev),
     because a plain `colcon build` copies this file into
     install/follow_ros2/lib/pythonX/site-packages/, at which point
     os.path.realpath no longer leads back to ros2_ws/src and this fallback
     silently finds nothing -- which is fine, because step 1 or 2 is the
     supported path for anything that isn't a laptop dev loop.

Call ensure_follow_core() before `import follow_core` anywhere in this
package.
"""
from __future__ import annotations

import importlib
import os
import sys

_DONE = False


def ensure_follow_core() -> None:
    global _DONE
    if _DONE:
        return
    _DONE = True

    try:
        importlib.import_module("follow_core")
        return
    except ImportError:
        pass

    candidates = []
    env_path = os.environ.get("FOLLOW_CORE_PATH")
    if env_path:
        candidates.append(env_path)

    here = os.path.dirname(os.path.realpath(__file__))
    probe = here
    for _ in range(8):
        probe = os.path.dirname(probe)
        if not probe or probe == os.path.dirname(probe):
            break
        candidates.append(probe)

    for base in candidates:
        marker = os.path.join(base, "follow_core", "__init__.py")
        if os.path.isfile(marker):
            if base not in sys.path:
                sys.path.insert(0, base)
            break

    # Re-raise with a clear message if it's still not there -- a bare
    # ModuleNotFoundError from deep inside a node is not actionable.
    try:
        importlib.import_module("follow_core")
    except ImportError as exc:
        raise ImportError(
            "follow_core is not importable. Set FOLLOW_CORE_PATH to the "
            "follow-drone repo root, pip install it, or add it to "
            "PYTHONPATH. See ros2_ws/src/follow_ros2/README.md."
        ) from exc
