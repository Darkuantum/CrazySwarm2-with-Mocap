#!/usr/bin/env bash
# WHICH crazyflie_server is live -- hardware, simulator, both, or none.
#
# Run this before ANY flight script. It exists because of a real incident
# (2026-10-07): a sim stack was started and a flight script run while a
# HARDWARE stack was already up for someone else's session. Both servers
# advertise the same /cfX/* service names on the same ROS domain, the client
# binds to whichever answers, and the script armed and flew the real drones.
#
# The mistake that allowed it was checking for the SIM server's process and
# treating "yes" as proof the real one was absent. It is not: the question is
# not "is sim running" but "is anything ELSE running".
#
#   ./scripts/which_server.sh                 # report and exit
#   ./scripts/which_server.sh --require sim   # exit non-zero unless ONLY sim
#   ./scripts/which_server.sh --require hw    # exit non-zero unless ONLY hardware
#
# Exit codes: 0 = matches --require (or just reporting), 1 = wrong backend,
#             2 = BOTH live (ambiguous -- never fly), 3 = none live.
set -uo pipefail

HW_PAT='install/crazyflie/lib/crazyflie/crazyflie_server'
SIM_PAT='install/crazyflie_sim/lib/crazyflie_sim/crazyflie_server'

# -f matches the full command line; exclude this script and its shell.
pids_for() { pgrep -f "$1" 2>/dev/null | grep -vx "$$" | grep -vx "${PPID:-0}" || true; }

HW=$(pids_for "$HW_PAT")
SIM=$(pids_for "$SIM_PAT")
REQUIRE="${2:-}"
[[ "${1:-}" == "--require" ]] || REQUIRE=""

if [[ -n "$HW" && -n "$SIM" ]]; then
  echo "BOTH a hardware and a simulator server are running."
  echo "  hardware: $(echo $HW | tr '\n' ' ')"
  echo "  sim     : $(echo $SIM | tr '\n' ' ')"
  echo "They advertise the SAME /cfX/* services. A flight script will bind to"
  echo "whichever answers first -- which may be the real drones. DO NOT FLY."
  exit 2
fi

if [[ -n "$HW" ]]; then
  echo "HARDWARE server is live (pid $(echo $HW | tr '\n' ' ')) -- commands reach REAL drones."
  BACKEND=hw
elif [[ -n "$SIM" ]]; then
  echo "SIMULATOR server is live (pid $(echo $SIM | tr '\n' ' ')) -- nothing physical will move."
  BACKEND=sim
else
  echo "No crazyflie_server is running. A flight script will hang waiting for /all/*."
  exit 3
fi

if [[ -n "$REQUIRE" ]]; then
  case "$REQUIRE" in
    sim) [[ "$BACKEND" == sim ]] || { echo "REFUSED: expected sim, found hardware."; exit 1; } ;;
    hw|hardware) [[ "$BACKEND" == hw ]] || { echo "REFUSED: expected hardware, found sim."; exit 1; } ;;
    *) echo "unknown --require '$REQUIRE' (use sim|hw)"; exit 1 ;;
  esac
  echo "matches --require $REQUIRE"
fi
exit 0
