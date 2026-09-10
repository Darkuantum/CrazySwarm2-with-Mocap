#!/usr/bin/env bash
# mocap_diag.sh -- offline-analysable diagnosis of the NatNet/OptiTrack data path.
#
# RUN THIS WHILE CONNECTED TO SSID "motive", WITH MOTIVE STREAMING.
# It writes everything to a single text file and needs no internet and no sudo.
#
#   ./mocap_diag.sh              # writes ./mocap_diag_<host>_<stamp>.txt
#   ./mocap_diag.sh /tmp/out.txt # explicit output path
#
# Then reconnect to the internet and hand the file over for analysis.
#
# It answers, in order:
#   1. what addresses/MACs this machine actually has          (settles interface_ip)
#   2. which interface the kernel would use for Motive        (routing)
#   3. whether the NatNet COMMAND socket works                (unicast :1510)
#   4. whether the NatNet DATA stream arrives, per interface  (multicast :1511)  <-- the real test
#   5. whether the real node then produces /poses             (unbuffered, so it prints)

# NOTE: deliberately no 'set -u'. ROS 2's setup.bash references unset variables,
# so -u makes every 'source /opt/ros/humble/setup.bash' abort its subshell.
set -o pipefail

MOTIVE_HOST="${MOTIVE_HOST:-192.168.9.124}"
MCAST_GROUP="${MCAST_GROUP:-239.255.42.99}"
DATA_PORT="${DATA_PORT:-1511}"
CMD_PORT="${CMD_PORT:-1510}"
LISTEN_SECS="${LISTEN_SECS:-6}"

OUT="${1:-$PWD/mocap_diag_$(hostname -s)_$(date +%Y%m%d-%H%M%S).txt}"
exec > >(tee "$OUT") 2>&1

sec() { echo; echo "==================== $* ===================="; }
run() { echo "\$ $*"; eval "$@" 2>&1 | sed 's/^/    /'; echo; }

echo "mocap_diag.sh"
echo "date        : $(date -Is)"
echo "host        : $(hostname)  ($(hostname -s))"
echo "kernel      : $(uname -r)"
echo "motive host : $MOTIVE_HOST"
echo "natnet      : cmd udp/$CMD_PORT   data mcast $MCAST_GROUP:$DATA_PORT"
echo "output      : $OUT"

sec "1. INTERFACES AND ADDRESSES  (interface_ip must be one of these)"
run "ip -o -f inet addr show"
echo "    -- MAC addresses (a DHCP static reservation is bound to the MAC) --"
for i in $(ls /sys/class/net | grep -v '^lo$'); do
  mac=$(cat "/sys/class/net/$i/address" 2>/dev/null)
  oper=$(cat "/sys/class/net/$i/operstate" 2>/dev/null)
  v4=$(ip -o -f inet addr show dev "$i" 2>/dev/null | awk '{print $4}' | paste -sd, -)
  printf "    %-14s mac=%s  state=%-6s ipv4=%s\n" "$i" "$mac" "$oper" "${v4:-none}"
done
echo
echo "    -- current Wi-Fi association --"
run "iw dev 2>/dev/null | grep -E 'Interface|ssid' || nmcli -t -f DEVICE,STATE,CONNECTION dev 2>/dev/null"

sec "2. ROUTING  (decides the join interface when interface_ip is 0.0.0.0)"
run "ip route show"
echo "    -- which source address/interface the kernel picks --"
run "ip route get $MOTIVE_HOST"
run "ip route get $MCAST_GROUP"
echo "    NOTE: the interface in 'ip route get $MCAST_GROUP' is the one a 0.0.0.0"
echo "          join would land on. If that is not the motive interface, the data"
echo "          socket subscribes on the wrong NIC and /poses stays silent."

sec "3. REACHABILITY OF THE MOTIVE PC  (necessary, NOT sufficient)"
run "ping -c 3 -W 2 $MOTIVE_HOST"
run "arp -n 2>/dev/null | head -20 || ip neigh show"
echo "    NOTE: ping only exercises the unicast path. It succeeds even when the"
echo "          multicast data socket is joined on the wrong interface."

sec "4. STALE LISTENERS ON THE DATA PORT"
run "ss -uanp 2>/dev/null | grep -E \":$DATA_PORT|:$CMD_PORT\" || echo '(none)'"
echo "    NOTE: two sockets on :$DATA_PORT means a leftover process is competing"
echo "          for the stream. Kill it before trusting anything below."

# Is someone already receiving the stream ON THIS MACHINE? If so, the binding
# tests below are not automatically safe: multicast fans out to every socket
# (harmless), but a UNICAST NatNet stream is delivered to exactly ONE socket,
# so binding :$DATA_PORT would steal frames from whoever is already there --
# and no mocap means fly-away. Skip the intrusive sections unless forced.
BUSY=0
if ss -uanp 2>/dev/null | grep -q ":$DATA_PORT "; then BUSY=1; fi
FORCE="${FORCE:-0}"
if [ "$BUSY" = 1 ] && [ "$FORCE" != 1 ]; then
  echo
  echo "    *** SOMETHING IS ALREADY BOUND TO :$DATA_PORT ON THIS MACHINE. ***"
  echo "    Sections 7 and 8 bind that port and will be SKIPPED, because if"
  echo "    Motive is streaming unicast they would steal frames from the"
  echo "    process that already has it -- possibly a drone in flight."
  echo "    Sections 1-6 and 9 are read-only and still ran."
  echo "    When the rig is free, re-run with:  FORCE=1 $0"
fi

sec "5. CURRENT IGMP MEMBERSHIPS"
run "cat /proc/net/igmp"
echo "    ($MCAST_GROUP reversed-hex is 632AFFEF; look for it above)"

sec "6. NATNET COMMAND SOCKET  (unicast udp/$CMD_PORT -> $MOTIVE_HOST)"
python3 - "$MOTIVE_HOST" "$CMD_PORT" <<'PY' 2>&1 | sed 's/^/    /'
import socket, struct, sys
host, port = sys.argv[1], int(sys.argv[2])
# NatNet NAT_CONNECT (id 0), payload = 256-byte name + version bytes
pkt = struct.pack('<HH', 0, 260) + b'diag'.ljust(256, b'\0') + bytes([4,1,0,0]) + bytes([4,1,0,0])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(3.0)
try:
    s.sendto(pkt, (host, port))
    print(f"sent NAT_CONNECT ({len(pkt)} bytes) to {host}:{port}")
    data, addr = s.recvfrom(65535)
    mid, ln = struct.unpack('<HH', data[:4])
    print(f"REPLY from {addr}: {len(data)} bytes, message_id={mid} len={ln}")
    if mid in (1, 5) and len(data) > 8:   # NAT_PINGRESPONSE=1 (modern), 5 on older builds
        name = data[4:260].split(b'\0')[0].decode('ascii', 'replace')
        print(f"  -> NAT_SERVERINFO, app='{name}'")
        tail = data[260:]
        if len(tail) >= 12:
            print(f"  -> version bytes: app={list(tail[0:4])} natnet={list(tail[4:8])}")
    print("VERDICT: command socket OK -- Motive is up and answering unicast.")
    # Be a good citizen on a SHARED Motive server: in unicast streaming mode a
    # connect request registers this host as a client. Hand the registration
    # back rather than leaving it to time out.
    try:
        s.sendto(struct.pack('<HH', 9, 0), (host, port))   # NAT_DISCONNECT
        print("sent NAT_DISCONNECT (left no stale client registration).")
    except OSError:
        pass
except socket.timeout:
    print("NO REPLY within 3s.")
    print("VERDICT: command socket FAILED. Wrong host, Motive not streaming,")
    print("         or a firewall. Fix this before looking at multicast.")
except OSError as e:
    print(f"SOCKET ERROR: {e}")
    print("VERDICT: no route to the Motive PC from this machine.")
finally:
    s.close()
PY

sec "7. MULTICAST DATA STREAM, PER CANDIDATE INTERFACE  (the decisive test)"
echo "    Joins $MCAST_GROUP:$DATA_PORT with IP_ADD_MEMBERSHIP pinned to each local"
echo "    address in turn and counts frames for ${LISTEN_SECS}s. The address that"
echo "    receives data is exactly what interface_ip must be set to."
echo
if [ "$BUSY" = 1 ] && [ "$FORCE" != 1 ]; then
  echo "    SKIPPED -- another process holds :$DATA_PORT (see section 4)."
else
python3 - "$MCAST_GROUP" "$DATA_PORT" "$LISTEN_SECS" <<'PY' 2>&1 | sed 's/^/    /'
import socket, struct, sys, time, errno, subprocess
group, port, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])

cands = []
out = subprocess.run(["ip","-o","-f","inet","addr","show"], capture_output=True, text=True).stdout
for line in out.splitlines():
    f = line.split()
    dev, cidr = f[1], f[3]
    ip = cidr.split('/')[0]
    if dev != 'lo' and not dev.startswith(('docker','br-','veth')):
        cands.append((dev, ip))
cands.append(('<kernel default route>', '0.0.0.0'))

if not cands:
    print("no candidate interfaces found")
for dev, ip in cands:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(('', port))
    except OSError as e:
        print(f"{dev:<22} {ip:<16} BIND FAILED: {e} (is the mocap node running?)")
        s.close(); continue
    try:
        mreq = struct.pack('4s4s', socket.inet_aton(group), socket.inet_aton(ip))
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    except OSError as e:
        name = errno.errorcode.get(e.errno, e.errno)
        print(f"{dev:<22} {ip:<16} JOIN FAILED: {name} ({e.strerror})")
        s.close(); continue
    s.settimeout(0.5)
    n = nbytes = 0; t0 = time.time(); first = None
    while time.time() - t0 < secs:
        try:
            d, a = s.recvfrom(65535)
        except socket.timeout:
            continue
        n += 1; nbytes += len(d)
        if first is None: first = a
    s.close()
    if n:
        print(f"{dev:<22} {ip:<16} *** {n} frames ({nbytes} B) in {secs:.0f}s "
              f"~{n/secs:.0f} Hz, from {first[0]}  <== USE THIS")
    else:
        print(f"{dev:<22} {ip:<16} join ok, but 0 frames in {secs:.0f}s")
print()
print("READING THIS:")
print(" * exactly one address receiving  -> set interface_ip to it.")
print(" * '<kernel default route>' 0.0.0.0 receives too -> the default route already")
print("   points at the motive NIC; interface_ip is belt-and-braces, not the fix.")
print(" * 0.0.0.0 gets 0 while a named address gets data -> THIS is the silent")
print("   starve: the default route is stealing the join. interface_ip fixes it.")
print(" * nothing anywhere, but section 6 said the command socket is OK -> Motive")
print("   is streaming UNICAST, not multicast. Fix it in Motive's streaming pane.")
print(" * JOIN FAILED ENODEV -> that address is not on this machine right now.")
PY
fi

sec "8. THE REAL NODE, UNBUFFERED  (its stdout is normally lost to block buffering)"
CFG=""
if [ "$BUSY" = 1 ] && [ "$FORCE" != 1 ]; then
  echo "    SKIPPED -- another process holds :$DATA_PORT (see section 4)."
  CFG="__skip__"
fi
for c in "$HOME/CrazySwarm2-with-Mocap/src/crazyswarm2/crazyflie_shows/config/motion_capture.yaml" \
         "$HOME/near-intern/swarm-shows/crazyflie_shows/config/motion_capture.yaml" \
         "$HOME/CrazySwarm2-with-Mocap/src/crazyswarm2/crazyflie/config/motion_capture.yaml"; do
  [ "$CFG" = "__skip__" ] && break
  [ -f "$c" ] && { CFG="$c"; break; }
done
if [ "$CFG" = "__skip__" ]; then
  :
elif [ -z "$CFG" ]; then
  echo "    no motion_capture.yaml found; skipping"
else
  echo "    config: $CFG"
  echo "    -- effective settings --"
  grep -nE "^\s*(type|hostname|interface_ip)\s*:" "$CFG" | sed 's/^/      /'
  echo
  echo "    REMINDER: type MUST be \"optitrack\". The apt build of"
  echo "    ros-humble-motion-capture-tracking has NO optitrack_closed_source"
  echo "    backend compiled in -- that value aborts instantly with"
  echo "    'Unknown motion capture type!'."
  echo
  TMP=$(mktemp /tmp/mocap_diag_params_XXXX.yaml)
  python3 - "$CFG" "$TMP" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
p = c['/motion_capture_tracking']['ros__parameters']
# rcl's parser rejects an empty mapping ("Cannot have a value before
# ros__parameters"), so strip empty dicts rather than emitting rigid_bodies: {}.
def strip_empty(d):
    return {k: (strip_empty(v) if isinstance(v, dict) else v)
            for k, v in d.items()
            if not (isinstance(v, dict) and not strip_empty(v))}
p = strip_empty(p)
yaml.safe_dump({'/motion_capture_tracking': {'ros__parameters': p}}, open(sys.argv[2], 'w'))
PY
  NODE=/opt/ros/humble/lib/motion_capture_tracking/motion_capture_tracking_node
  if [ -x "$NODE" ]; then
    echo "    running $NODE for 10s (stdbuf -oL, so it actually prints):"
    ( source /opt/ros/humble/setup.bash >/dev/null 2>&1
      stdbuf -oL -eL timeout 10 "$NODE" --ros-args -r __node:=motion_capture_tracking \
        --params-file "$TMP" ) 2>&1 | sed 's/^/      /'
    rc=${PIPESTATUS[0]}
    echo "    exit status: $rc"
    case $rc in
      124) echo "    -> STILL RUNNING at timeout. Either working, or hung in recv."
           echo "       'receive_from: Interrupted system call' on teardown only means"
           echo "       it was sitting in recv; it does NOT mean data was arriving."
           echo "       Section 7 is what decides that."
           echo "       NB: the command socket is contacted BEFORE the multicast join,"
           echo "       so if Motive is unreachable the node hangs there and a wrong"
           echo "       interface_ip is never reported at all." ;;
      134) echo "    -> SIGABRT. Read the 'what():' line above: 'Unknown motion capture"
           echo "       type!' = bad type string; 'Invalid argument' = hostname is not a"
           echo "       literal dotted-quad IP (this backend does NOT resolve names)." ;;
      139) echo "    -> SIGSEGV. Unexplained crash; capture and report." ;;
      *)   echo "    -> see above" ;;
    esac
  else
    echo "    node binary not found at $NODE"
  fi
  rm -f "$TMP"
fi

sec "9. /poses UNDER THE FULL STACK  (optional, needs the stack already running)"
if command -v ros2 >/dev/null 2>&1; then
  ( source /opt/ros/humble/setup.bash >/dev/null 2>&1
    [ -f "$HOME/CrazySwarm2-with-Mocap/install/setup.bash" ] && \
      source "$HOME/CrazySwarm2-with-Mocap/install/setup.bash" >/dev/null 2>&1
    echo "    ros2 pkg prefix crazyflie -> $(ros2 pkg prefix crazyflie 2>&1)"
    echo "    -- topic hz /poses (8s) --"
    timeout 8 ros2 topic hz /poses 2>&1 | head -6 | sed 's/^/      /'
    echo "    -- publisher info --"
    timeout 5 ros2 topic info /poses 2>&1 | sed 's/^/      /'
  )
else
  echo "    ros2 not on PATH; skipped"
fi

sec "DONE"
echo "Wrote: $OUT"
echo "Reconnect to the internet and hand this file over."
