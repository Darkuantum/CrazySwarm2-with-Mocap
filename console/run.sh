#!/usr/bin/env bash
# Start the CrazySwarm2 mission console.
#
#   ./console/run.sh                 # http://localhost:8077, opens a browser
#   ./console/run.sh --port 9000
#   ./console/run.sh --host 0.0.0.0  # reachable from another machine on the lab LAN
#   ./console/run.sh --no-browser
#
# This script exists so the console's child processes inherit exactly the
# environment you would have in a terminal -- the commands the UI shows you are
# then literally the commands you could type. It does the three things this
# workspace always needs:
#
#   1. sources ROS with `set +u` (ROS's setup.bash reads the unbound
#      AMENT_TRACE_SETUP_FILES and aborts the script under `set -u`),
#   2. sources this workspace's overlay so the VENDORED motion_capture_tracking
#      wins over the broken apt one,
#   3. strips conda/venv from PATH and PYTHONPATH -- ROS 2 runs its nodes with
#      /usr/bin/python3, and a conda interpreter produces "No module named
#      _cffirmware" and invalid message types.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

# 3. de-conda first, so the python we then use for everything is the system one.
if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
  echo "console: stripping conda/venv from PATH (ROS needs /usr/bin/python3)"
  strip_path() {
    local out="" p
    IFS=':' read -ra parts <<< "${1:-}"
    for p in "${parts[@]}"; do
      [[ -z "$p" ]] && continue
      [[ -n "${CONDA_PREFIX:-}" && "$p" == "$CONDA_PREFIX"* ]] && continue
      [[ -n "${VIRTUAL_ENV:-}" && "$p" == "$VIRTUAL_ENV"* ]] && continue
      [[ "$p" == *"/conda/"* || "$p" == *"/anaconda"* || "$p" == *"/miniconda"* ]] && continue
      out="${out:+$out:}$p"
    done
    printf '%s' "$out"
  }
  PATH="$(strip_path "$PATH")"
  PYTHONPATH="$(strip_path "${PYTHONPATH:-}")"
  unset CONDA_PREFIX VIRTUAL_ENV
  export PATH PYTHONPATH
fi

# 1. ROS distro.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for d in jazzy humble; do
    [[ -f "/opt/ros/$d/setup.bash" ]] && ROS_DISTRO="$d" && break
  done
fi
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  set +u; source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
else
  echo "console: WARNING - no /opt/ros/<distro>/setup.bash found."
  echo "console: the UI will still start, but every ROS check will report the missing environment."
fi

# 2. this workspace's overlay.
if [[ -f "$REPO/install/setup.bash" ]]; then
  set +u; source "$REPO/install/setup.bash"; set -u
else
  echo "console: WARNING - $REPO/install/setup.bash missing; run ./scripts/build.sh first."
fi

cd "$REPO"
export PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m mission_console "$@"
