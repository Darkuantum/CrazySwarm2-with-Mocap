#!/usr/bin/env bash
#
# Scan every ENABLED drone's radio address, derived from crazyflies.yaml.
#
# WHY THIS EXISTS AS A SCRIPT
# ---------------------------
# The cpp server connects drones in lexicographic order and hangs **silently**
# on the first enabled drone that does not answer: no error, no `/all/*`
# services, and it needs SIGKILL. So "scan every enabled address before every
# launch" is the go/no-go rule for this rig.
#
# That rule used to be written out as a literal list of addresses in five
# documents, which drifted apart: on 2026-10-06 CLAUDE.md said
# 01/02/03/05/08, while README, RUNNING, TROUBLESHOOTING and MOCAP all said
# 01/02/03/10/14 -- and cf10/cf14 did not exist in the yaml at all. The one
# drone no document named, cf4, is the one the yaml marks as intermittent.
# A roster lives in exactly one place (crazyflies.yaml); every consumer
# derives it. Do not re-introduce a literal list.
#
# Usage:
#   scripts/scan_fleet.sh              # scan every enabled address
#   scripts/scan_fleet.sh --list       # just print what WOULD be scanned
#   scripts/scan_fleet.sh --all        # include drones with enabled: false
#   scripts/scan_fleet.sh --yaml PATH  # a one-off fleet file
#
# Needs the workspace sourced (`source install/setup.bash`) for `ros2 run
# crazyflie scan`. Stop the server first: one process owns a dongle at a time.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
YAML="$REPO/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
LIST_ONLY=0
WANT_ALL=0

while [ $# -gt 0 ]; do
  case "$1" in
    --list) LIST_ONLY=1; shift ;;
    --all)  WANT_ALL=1;  shift ;;
    --yaml) YAML="$2";   shift 2 ;;
    -h|--help) sed -n '3,27p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
  esac
done

[ -r "$YAML" ] || { echo "cannot read $YAML" >&2; exit 1; }

# name<TAB>address<TAB>uri<TAB>enabled, in lexicographic order -- the SAME
# order the server connects in, so the first failure here is the first drone
# that would hang the launch.
ROWS="$(WANT_ALL="$WANT_ALL" python3 - "$YAML" <<'PY'
import os
import sys

import yaml

robots = (yaml.safe_load(open(sys.argv[1])) or {}).get('robots') or {}
want_all = os.environ.get('WANT_ALL') == '1'
for name in sorted(robots):
    r = robots[name] or {}
    on = bool(r.get('enabled'))
    if not on and not want_all:
        continue
    uri = r.get('uri', '')
    addr = uri.rsplit('/', 1)[-1] if uri else ''
    if not addr:
        print(f'{name}\t\t{uri}\t{on}')
        continue
    print(f'{name}\t0x{addr}\t{uri}\t{on}')
PY
)"

if [ -z "$ROWS" ]; then
  echo "no $( [ "$WANT_ALL" = 1 ] && echo '' || echo 'enabled ')drones in $YAML" >&2
  exit 1
fi

echo "fleet from $YAML"
printf '%s\n' "$ROWS" | while IFS=$'\t' read -r name addr uri on; do
  printf '  %-6s %-16s %-28s %s\n' "$name" "$addr" "$uri" \
    "$([ "$on" = True ] && echo enabled || echo 'DISABLED')"
done
echo

if [ "$LIST_ONLY" = 1 ]; then
  exit 0
fi

# A URI that pins a datarate the drone does not speak hangs the server forever
# with no error, so the scan result must be read for the DATARATE too, not
# just for "it answered".
fail=0
printf '%s\n' "$ROWS" | {
  while IFS=$'\t' read -r name addr uri on; do
    [ -n "$addr" ] || { echo "-- $name: no address in its uri ($uri)"; fail=1; continue; }
    echo "-- $name  $addr   (uri pins ${uri##*//})"
    if out="$(ros2 run crazyflie scan --address "$addr" 2>&1)"; then
      printf '%s\n' "$out" | sed 's/^/     /'
      printf '%s\n' "$out" | grep -q 'radio://' || {
        echo "     *** NO ANSWER -- this drone will hang the server ***"
        fail=1
      }
    else
      printf '%s\n' "$out" | sed 's/^/     /'
      echo "     *** scan failed ***"
      fail=1
    fi
  done
  exit "$fail"
} || fail=1

echo
if [ "$fail" = 0 ]; then
  echo "GO: every address answered. Check each answer's datarate matches its uri above."
else
  echo "NO-GO: an address did not answer. The server will hang silently on it."
  echo "       Either power-cycle/replace that drone, or set enabled: false in"
  echo "       $YAML (and re-run sync_initial_positions.py if the fleet changed)."
fi
exit "$fail"
