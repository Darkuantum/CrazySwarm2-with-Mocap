#!/usr/bin/env bash
# mocap_verify.sh -- prove /poses actually publishes, and see what Motive names
# its rigid bodies. NEEDS NO DRONES, NO CRAZYRADIO, NO FLYING.
#
# Run on SSID "motive" with Motive streaming:
#   ./mocap_verify.sh              # writes ./mocap_verify_<host>_<stamp>.txt
#
# This is the check that mocap_diag.sh could not do: diag proves frames reach
# the NIC, this proves the ROS node turns them into /poses, at what rate, and
# under which rigid-body NAMES. Those names must match the drone keys in
# crazyflies.yaml (cf1, cf2, cf3, cf10, cf12) or the server sees nothing.

set -o pipefail   # no 'set -u': ROS setup.bash trips it

OUT="${1:-$PWD/mocap_verify_$(hostname -s)_$(date +%Y%m%d-%H%M%S).txt}"
exec > >(tee "$OUT") 2>&1
sec() { echo; echo "==================== $* ===================="; }

NODE=/opt/ros/humble/lib/motion_capture_tracking/motion_capture_tracking_node
CFG=""
for c in "$HOME/CrazySwarm2-with-Mocap/src/crazyswarm2/crazyflie_shows/config/motion_capture.yaml" \
         "$HOME/near-intern/swarm-shows/crazyflie_shows/config/motion_capture.yaml"; do
  [ -f "$c" ] && { CFG="$c"; break; }
done

echo "mocap_verify.sh"
echo "date   : $(date -Is)"
echo "config : ${CFG:-NONE FOUND}"
echo "output : $OUT"
[ -z "$CFG" ] && { echo "no config found, aborting"; exit 1; }
[ -x "$NODE" ] || { echo "node binary missing at $NODE, aborting"; exit 1; }

sec "0. PRECONDITIONS"
grep -nE "^\s*(type|hostname|interface_ip)\s*:" "$CFG" | sed 's/^/    /'
MYIP=$(ip -o -f inet addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -v '^127\.')
echo "    this machine: $(echo $MYIP | tr '\n' ' ')"
# interface_ip in the yaml is INERT in ros-humble-motion-capture-tracking 1.0.9:
# the node never forwards it, and always joins on the hardcoded 141.23.110.162.
# That address is therefore the real precondition. (strace-proven 2026-09-07.)
HARDCODED=141.23.110.162
if echo "$MYIP" | grep -qx "$HARDCODED"; then
  echo "    OK: $HARDCODED is present -- the hardcoded multicast join can succeed."
else
  echo "    *** $HARDCODED is NOT on this machine. ***"
  echo "    *** 1.0.9 ignores interface_ip and ALWAYS joins on that address, so"
  echo "    *** the node will abort with 'set_option: No such device'."
  DEV=$(ip -o -f inet addr show 2>/dev/null | awk '$4 ~ /^192\.168\.9\./ {print $2; exit}')
  DEV=${DEV:-wlp131s0f0}
  echo "    *** Fix (re-apply every session; NM drops it on reconnect):"
  echo "    ***   sudo ip addr add $HARDCODED/32 dev $DEV"
fi
if ss -uanp 2>/dev/null | grep -q ":1511 "; then
  echo "    *** WARNING: something already holds :1511. Aborting to stay safe. ***"
  exit 1
fi

TMP=$(mktemp /tmp/mocap_verify_XXXX.yaml)
python3 - "$CFG" "$TMP" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
p = c['/motion_capture_tracking']['ros__parameters']
def strip_empty(d):
    return {k: (strip_empty(v) if isinstance(v, dict) else v)
            for k, v in d.items() if not (isinstance(v, dict) and not strip_empty(v))}
yaml.safe_dump({'/motion_capture_tracking': {'ros__parameters': strip_empty(p)}},
               open(sys.argv[2], 'w'))
PY

sec "1. START THE NODE (25s, unbuffered)"
LOG=$(mktemp /tmp/mocap_verify_node_XXXX.log)
( source /opt/ros/humble/setup.bash >/dev/null 2>&1
  stdbuf -oL -eL timeout 25 "$NODE" --ros-args -r __node:=motion_capture_tracking \
    --params-file "$TMP" ) > "$LOG" 2>&1 &
NODEPID=$!
for i in $(seq 1 12); do grep -q . "$LOG" && break; done
sleep 4
echo "    node stdout so far:"
sed 's/^/      /' "$LOG"
if ! kill -0 $NODEPID 2>/dev/null; then
  echo "    *** NODE ALREADY DEAD -- read the what(): line above. ***"
fi

sec "2. IS /poses PUBLISHING?"
( source /opt/ros/humble/setup.bash >/dev/null 2>&1
  [ -f "$HOME/CrazySwarm2-with-Mocap/install/setup.bash" ] && \
    source "$HOME/CrazySwarm2-with-Mocap/install/setup.bash" >/dev/null 2>&1
  echo "    -- topic info --"
  timeout 6 ros2 topic info /poses 2>&1 | sed 's/^/      /'
  echo "    -- topic hz over 10s (THE number that matters) --"
  timeout 12 ros2 topic hz /poses 2>&1 | head -8 | sed 's/^/      /'
)

sec "3. WHICH RIGID BODIES IS MOTIVE STREAMING?"
echo "    These names must match the robot keys in crazyflies.yaml."
( source /opt/ros/humble/setup.bash >/dev/null 2>&1
  [ -f "$HOME/CrazySwarm2-with-Mocap/install/setup.bash" ] && \
    source "$HOME/CrazySwarm2-with-Mocap/install/setup.bash" >/dev/null 2>&1
  timeout 10 ros2 topic echo /poses --once > /tmp/poses_once.$$ 2>&1
  python3 - /tmp/poses_once.$$ "$HOME/near-intern/swarm-shows/crazyflie_shows/config/crazyflies.yaml" <<'PY'
import sys, re, yaml, math
txt = open(sys.argv[1]).read()
# name: X  followed by position x/y/z
bodies = []
for m in re.finditer(r"^\s*-\s*name:\s*(\S+).*?position:\s*\n\s*x:\s*(-?[\d.e+]+)\s*\n\s*y:\s*(-?[\d.e+]+)\s*\n\s*z:\s*(-?[\d.e+]+)", txt, re.S | re.M):
    bodies.append((m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4))))
if not bodies:
    print("  no rigid bodies parsed; raw head follows:"); print(txt[:800]); sys.exit()
print(f"  Motive is streaming {len(bodies)} rigid bodies:")
for n, x, y, z in bodies:
    print(f"    {n:8} ({x:7.3f}, {y:7.3f}, {z:6.3f})")
try:
    cfg = yaml.safe_load(open(sys.argv[2]))
except Exception:
    sys.exit()
enabled = {k: v['initial_position'] for k, v in cfg['robots'].items() if v['enabled']}
seen = {n: (x, y) for n, x, y, z in bodies}
print()
print("  CROSS-CHECK against enabled drones in crazyflies.yaml:")
for k, ip in enabled.items():
    if k not in seen:
        print(f"    {k:8} *** NOT STREAMED BY MOTIVE -- this drone would get no pose ***")
    else:
        d = math.dist(seen[k], ip[:2])
        flag = "" if d < 0.15 else f"  <-- yaml initial_position is {d:.2f} m away"
        print(f"    {k:8} streamed{flag}")
extra = [n for n in seen if n not in enabled]
if extra:
    print(f"    streamed but not enabled here: {', '.join(sorted(extra))}")
PY
  rm -f /tmp/poses_once.$$
)

sec "4. NODE OUTPUT AT SHUTDOWN"
wait $NODEPID 2>/dev/null
echo "    exit status: $?"
sed 's/^/      /' "$LOG"
echo
echo "    'receive_from: Interrupted system call' at the END is the normal"
echo "    teardown abort -- it says nothing about whether data was flowing."
echo "    Section 2 is what decides that."

rm -f "$TMP" "$LOG"
sec "DONE"
echo "Wrote: $OUT"
