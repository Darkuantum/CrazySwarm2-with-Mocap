#!/usr/bin/env bash
# Stop the CrazySwarm2 stack the way it actually needs stopping.
#
#   ./console/stop_stack.sh            # graceful: SIGINT, escalate, verify
#   ./console/stop_stack.sh --hard     # skip straight to SIGKILL
#
# Why this exists instead of a bare `pkill`:
#
#   * `ros2 launch` exits on SIGINT but its children DO NOT always follow. On
#     this rig the mocap node, RViz and the preflight GUI routinely survive,
#     and the mocap node is the one that matters -- it is blocked in recv(),
#     ignores SIGINT and SIGTERM, and keeps UDP 1511 bound. The next launch
#     then starves silently (two SO_REUSEPORT sockets on 1511 -> each datagram
#     goes to only ONE of them) with no error at all.
#   * So the only stop worth trusting is one that VERIFIES 1511 is released
#     afterwards. That check is the whole point of this script.
#
# It never touches the mission console itself, or its own process tree.
set -uo pipefail

HARD=0
[[ "${1:-}" == "--hard" ]] && HARD=1

SELF=$$
PARENT=${PPID:-0}

# Ordered most-parent first: killing the launcher first lets it take its own
# children down cleanly, so the later patterns usually match nothing.
PATTERNS=(
  'bin/ros2 launch crazyflie'
  'crazyflie_sim/lib/crazyflie_sim/crazyflie_server'
  'crazyflie/lib/crazyflie/crazyflie_server'
  'motion_capture_tracking_node'
  'preflight_kalman_plotter.py'
  'foxglove_bridge'
  'lib/rviz2/rviz2'
)

pids_for() {
  # -f matches the full command line, so exclude this script and its shell or
  # we kill ourselves mid-run (learned the hard way).
  pgrep -f "$1" 2>/dev/null | grep -vx "$SELF" | grep -vx "$PARENT" || true
}

all_pids() {
  local p out=""
  for p in "${PATTERNS[@]}"; do out+="$(pids_for "$p") "; done
  echo $out | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u || true
}

signal_round() {
  local sig=$1 wait_s=$2 pids
  pids=$(all_pids)
  [[ -z "$pids" ]] && return 0
  echo "  $sig -> $(echo $pids | tr '\n' ' ')"
  for p in $pids; do kill -"$sig" "$p" 2>/dev/null || true; done
  local waited=0
  while [[ $waited -lt $wait_s ]]; do
    sleep 1; waited=$((waited + 1))
    [[ -z "$(all_pids)" ]] && return 0
  done
  return 0
}

echo "stopping the crazyflie stack"
start_pids=$(all_pids)
if [[ -z "$start_pids" ]]; then
  echo "  nothing of the stack is running"
else
  echo "  found: $(echo $start_pids | tr '\n' ' ')"
  if [[ $HARD -eq 1 ]]; then
    signal_round KILL 3
  else
    signal_round INT 6
    signal_round TERM 4
    signal_round KILL 3
  fi
fi

left=$(all_pids)
echo
if [[ -n "$left" ]]; then
  echo "STILL RUNNING after SIGKILL: $(echo $left | tr '\n' ' ')"
  ps -o pid=,cmd= -p $(echo $left | tr '\n' ' ') 2>/dev/null | sed 's/^/    /'
else
  echo "all stack processes stopped"
fi

# The check that makes this worth running: a leftover here silently starves the
# NEXT launch's mocap node. Ping to Motive proves nothing (unicast != multicast).
echo
if ss -uanp 2>/dev/null | grep -q ':1511'; then
  echo "UDP 1511 STILL BOUND -- the next launch will starve:"
  ss -uanp 2>/dev/null | grep ':1511' | sed 's/^/    /'
  echo "  fix: ./console/stop_stack.sh --hard"
  exit 1
fi
echo "UDP 1511 clear -- the next launch can connect"

if lsusb 2>/dev/null | grep -qi '1915:7777'; then
  echo "Crazyradio present on USB and no server holding it -- the radio is free"
fi
exit 0
