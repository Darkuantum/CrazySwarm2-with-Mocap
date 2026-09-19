# HANDOVER — crazyflie_shows

**Written 2026-09-07.** Read with `CLAUDE.md` in the main repo, which describes
the rig in general. This file records what is *specific to this package* and
what is *still unresolved on the rig*.

It lives inside the package deliberately, so the rig knowledge travels with the
code. **Since 2026-09-18 the package lives in this workspace** at
`src/crazyswarm2/crazyflie_shows/` and is edited here. It was developed
out-of-tree in `~/near-intern/swarm-shows/` (single commit `714affe`); that copy
is retired — do not `cp -r` it back over this one, its roster is stale.

---

## 0. Where things stand — 2026-09-07

| | Status |
|---|---|
| Mocap `/poses` | ✅ **working, root-caused.** Needs `sudo ip addr add 141.23.110.162/32 dev wlp131s0f0` **every session** — §4a |
| `crazyflies.yaml` | ✅ five drones enabled (cf1, cf2, cf3, cf10, cf12), planner verified offline — §3 |
| Diagnostics | ✅ `scripts/mocap_diag.sh` — no drones, no radio, no sudo (`mocap_verify.sh` removed 2026-09-18, see §4a) |
| Flight (hardware) | ❌ **nothing has ever flown on hardware** |
| Flight (sim) | ✅ `swarm_show` flies end to end on `backend:=sim` — 2026-09-07, after fixing a `crazyflie_sim` bug (§4d) |
| Show algorithm | ✅ **done** — a ~62 s five-drone show. See `SHOW_GUIDE.md` |

**If you are picking this up cold, read §4a first.** The single fact that
unblocks the rig: `motion_capture_tracking` 1.0.9 ignores `interface_ip` and
always joins the multicast group on the hardcoded `141.23.110.162`, so that
address must be on the motive NIC or the node aborts at startup.

Needs the hardware, still open: scan all five radio addresses before any launch
(§6), confirm `cf12` vs `cf14` physically (§4c), and check `initial_position`
against where the drones actually sit before flying (§6). Full list in §8.

---

## 1. What this package is

A self-contained ROS 2 package for swarm choreography, built on `crazyflie_py`.
Developed out-of-tree in `~/near-intern/swarm-shows/`, folded into this
workspace on 2026-09-18.

**It is relocatable by construction** — verified, no absolute paths, no `../..`.
Four rules keep it that way; break any of them and it stops being relocatable:

1. resolve every resource via `get_package_share_directory()`
2. declare every dependency in `package.xml`
3. install resources to `share/` from `CMakeLists.txt`
4. never reference a path outside the package

```
crazyflie_shows/
├── package.xml CMakeLists.txt setup.py setup.cfg   packaging (mirrors crazyflie_examples)
├── HANDOVER.md       this file — the rig
├── SHOW_GUIDE.md     the show — figures, fill-ins, deployment, tuning
├── crazyflie_shows/
│   ├── figures.py        slot layouts, easing, trajectory fitting
│   ├── safety.py         offline separation / envelope checks, assignment
│   ├── choreography.py   THE SHOW — config, plan construction, verification
│   ├── plan_show.py      offline verifier + report        entry point
│   ├── swarm_show.py     the flight script                entry point
│   └── demo_show.py      the original starter show        entry point
├── config/           this package's crazyflies.yaml + motion_capture.yaml
├── launch/           show_launch.py — wraps crazyflie/launch.py
├── data/             trajectory CSVs
├── scripts/          mocap_diag.sh — on-rig NatNet diagnosis (see §4a)
└── reference/        firmware log + param TOC dumps (see §5)
```

`setup.py` is intentionally **empty** — `ament_cmake_python` generates its own
in the build tree and reads the entry points from `setup.cfg`. This mirrors
`crazyflie_examples` exactly. Do not "fix" it.

### Design split

`figures.py`, `safety.py` and `choreography.py` are **pure** — no rclpy, no
radio, no mocap. They import numpy and two classes from
`crazyflie_py.uav_trajectory` and nothing else, and `figures.py` loads that
module around `crazyflie_py`'s own rclpy import so the whole planner runs on a
laptop with no ROS installed. That is what makes the plan testable before
anything is armed.

The line that matters is between `choreography.py` and `swarm_show.py`.
`choreography.build_plan()` produces the entire show — every trajectory, every
phase, every position at every instant — and verifies it as it builds, raising
rather than warning. **Both `plan_show` and `swarm_show` call it**, so the
laptop verifies the same object the swarm flies, not a model of it.
`swarm_show.py` contains no geometry at all.

Keep choreography out of `figures.py`/`safety.py`, and keep geometry out of
`swarm_show.py`.

---

## 2. Build and run

```bash
cd ~/CrazySwarm2-with-Mocap          # the package is already in src/crazyswarm2/
./scripts/build.sh crazyflie_shows
source install/setup.bash
ros2 pkg prefix crazyflie      # MUST print this workspace, never /opt/ros/humble
```

That last line is not optional — see §4.

```bash
# verify on the ground first — no radio, no mocap, no drones
ros2 run crazyflie_shows plan_show
ros2 run crazyflie_shows plan_show --plot

# sim next, always. rviz:=False skips the visualiser.
ros2 launch crazyflie_shows show_launch.py backend:=sim
ros2 run crazyflie_shows swarm_show --ros-args -p use_sim_time:=true

# hardware: stack up, check the preflight GUI, then run the show by hand
ros2 launch crazyflie_shows show_launch.py
ros2 run crazyflie_shows swarm_show
```

Full procedure, including everything that must be filled in for this rig, is
in `SHOW_GUIDE.md`.

`use_sim_time` is required in sim: the sim clock runs ~4x slower than wall
time, so without it the script races ahead of the physics.

`show_launch.py` takes `show:=swarm_show` to auto-run the show as part of the
launch, and `rviz:=False` to skip rviz2. It defaults to empty on purpose — on hardware you want the stack up
and the preflight GUI checked *before* anything arms.

### When you must rebuild

The workspace builds with `--symlink-install`, so `install/` is a tree of
symlinks back into `src/`. That gives an asymmetric rule:

| Change | Rebuild? | Restart launch? |
|---|---|---|
| body of an existing registered `.py` | **no** | no |
| `config/*.yaml` | **no** | **yes** (yaml read once at launch) |
| new `.py` + new `setup.cfg` entry point | **yes** | no |
| new data file | **yes** (symlink must be created) | no |
| `.cpp` | yes | yes |
| `.srv` / `.msg` | yes, + every dependent package | yes |

`No executable found` almost always means you added a file and skipped the
rebuild.

### Config overriding

`config/crazyflies.yaml` and `config/motion_capture.yaml` are passed to
`crazyflie/launch.py` as **launch arguments**, so the vendored copies are never
edited. `server.yaml` and the URDF are **not** overridable — they are hardcoded
in that file's `parse_yaml()` and always come from the `crazyflie` package.

> **`config/crazyflies.yaml` has five drones enabled** (2026-09-07):
> `cf1, cf2, cf3, cf10, cf12` — the same five §3's planner was verified against.
> All five share one radio (`radio://0/80/2M/...`, dongle 0). **Scan every
> address before every launch** (§6): one unreachable drone wedges the whole
> server silently. `cf6` stays disabled (dead on radio); `cf5`/`cf11` stay
> commented out.

---

## 3. Verified state of the planner

Ran against the five committed `initial_position` values (cf1, cf2, cf3, cf10,
cf12), 2026-09-07, offline:

```
5 drones, swarm center = (0.068, -0.109)
best n-gon phase gather: min sep = 0.837 m   perm=(3, 0, 2, 4, 1)
  fixed phase  0 deg -> 0.804 m | 45 deg -> 0.811 m | 90 deg -> 0.806 m
circle closes: |eval(0)-eval(T)| = 1.33e-09 m, 6 pieces
spin envelope: 0.50 m/s tangential, 0.32 m/s^2 centripetal
check_leg correctly rejects an unsafe leg
```

The computed swarm centre `(0.068, -0.109)` sits next to the hand-measured
`ROOM_CENTER (0.0467, -0.1037)` in `multi_trajectory_formation.py` — an
independent check on both. The phase sweep buys ~4% of separation over a fixed
phase: modest, but free.

**None of this has flown.** It is verified geometry, not verified flight.

---

## 4. Rig issues — still open

### 4a. Mocap `/poses` — ROOT CAUSE FOUND 2026-09-07

> **`141.23.110.162` was right all along.** An earlier pass of this file (same
> day) declared that question "closed — no mechanism". **That was wrong, and it
> is retracted.** `strace` on the installed binary shows the mechanism plainly.

#### The root cause

`ros-humble-motion-capture-tracking` **1.0.9** declares the `interface_ip`
parameter but **never forwards it** to `MotionCaptureOptitrack()`. The library's
own default — `141.23.110.162`, a TU-Berlin address baked into the binary — is
used for the multicast join instead. Measured, with the parameter set to three
different values:

```
# params file said interface_ip: 0.0.0.0 / 192.168.8.196 / 1.2.3.4 — all three:
setsockopt(23, SOL_IP, IP_ADD_MEMBERSHIP,
           {imr_multiaddr=inet_addr("239.255.42.99"),
            imr_interface=inet_addr("141.23.110.162")}, 8) = -1 ENODEV
```

```
$ strings motion_capture_tracking_node | grep 141
141.23.110.162
```

So on this rig:

- **`interface_ip` in `motion_capture.yaml` does nothing.** Setting it to the
  "correct" address changes not one byte of the syscall.
- The node **always** joins on `141.23.110.162`. If that address is not present
  on this machine, `IP_ADD_MEMBERSHIP` returns **ENODEV**, boost throws, and the
  node aborts with `set_option: No such device`.
- Therefore **mocap cannot work at all** unless `141.23.110.162` is on a local
  interface that can reach Motive's multicast.

#### What actually happened on 2026-09-04

Both of these had to become true, and both did, at 18:12:

| Time | Change | Why it mattered |
|---|---|---|
| 18:10:08 | `ip addr add 141.23.110.162/32 dev wlp131s0f0` | makes the hardcoded join succeed |
| 18:12:19 | `type` set back to `"optitrack"` | `optitrack_closed_source` is not compiled in (below) |

Runs at 18:10:11 and 18:11:40 still died at 1.1 s because `type` was wrong from
18:09:22 to 18:12:19. Every run from 18:12:21 on stayed up. The address was not
withdrawn again. **The user's recollection that the drone started receiving
mocap is corroborated by the logs** — it began working at 18:12:19.

Earlier attempts failed because NetworkManager had withdrawn the address (at
18:07:29 on carrier loss, and 18:09:13 on Wi-Fi reconfigure), which is exactly
why it looked intermittent and unexplainable.

#### The fix

Put the address the node insists on onto the motive NIC:

```bash
sudo ip addr add 141.23.110.162/32 dev wlp131s0f0
ip -o -f inet addr show dev wlp131s0f0     # expect BOTH 192.168.9.x and 141.23.110.162
```

**This is deliberately NOT made permanent** (operator's choice, 2026-09-07), so
it is a **per-session step** — and it must be redone after any Wi-Fi reconnect
or carrier blip, because NetworkManager withdraws it. That withdrawal happened
twice on 09-04 (18:07:29 and 18:09:13) and is the single reason the problem
looked intermittent and unexplainable.

Two consequences worth holding onto:

- **Check before every launch, not just the first.** (Historical: the vendored
  driver no longer needs this address. `mocap_verify.sh`, which tested for it,
  was removed 2026-09-18 — it ran the apt node, which this rig no longer has.)
  A missing address is at least loud (the node aborts at startup).
- **A mid-session Wi-Fi reconnect is a live risk.** If the address goes away
  while the stack is up, you are in the fly-away case from §6, not the loud
  case. If the Wi-Fi drops during a session, re-check the address and restart
  the stack before arming anything.

`nmcli connection modify motive +ipv4.addresses 141.23.110.162/32` would make it
survive reconnects, if that is ever wanted.

The clean long-term fix is to build `motion_capture_tracking` from source in the
workspace with the constructor call corrected, at which point `interface_ip`
becomes real. Until then, treat the yaml key as inert.

#### Second, independent bug: `type: "optitrack_closed_source"`

1.0.9 ships six backends and that is not one of them:

```
$ strings .../motion_capture_tracking_node | grep -oE 'N16libmotioncapture[0-9]+[A-Za-z]+E' | sort -u
  MotionCapture  Mock  Vrpn  Vicon  FZMotion  Qualisys  Optitrack
```

Selecting it throws `Unknown motion capture type!` and aborts in ~1 s.
VS Code local history (`~/.config/Code/User/History/-797892e5/`) shows the value
toggled **eight times** on 09-04. Keep it at `"optitrack"`.

Also: this backend does **not resolve hostnames**. `hostname` must be a literal
dotted-quad or it aborts with `boost ... Invalid argument`.

#### Reading the launch logs correctly

Three distinct death signatures, and telling them apart is the whole trick:

| Signature | Meaning |
|---|---|
| SIGABRT ~1.1 s, `Unknown motion capture type!` | bad `type` string |
| SIGABRT ~1.1 s, `set_option: No such device` | `141.23.110.162` missing (the root cause) |
| SIGABRT ~0.9 s **after** Ctrl-C, `receive_from: Interrupted system call` | normal teardown — says nothing either way |
| SIGSEGV ~1.1 s (16:26–17:16 on 09-04 only) | **still unexplained**, not reproduced |

Row 3 is the trap: a hung node and a working node both sit in `recv` and abort
identically on SIGINT. While hung the node advertises only `/parameter_events`
and `/rosout` — **no `/poses`**. Check the topic, never the process.

#### END-TO-END CONFIRMED, 2026-09-07 16:28

With `141.23.110.162` added to `wlp131s0f0`, the node printed the line that had
been invisible for two sessions, and `/poses` published for the first time on
record:

```
Joined multicast group 239.255.42.99 on interface 141.23.110.162
/poses  Type: motion_capture_tracking_interfaces/msg/NamedPoseArray, Publisher count: 1
```

That closes the mocap question. Two observations from that run are recorded
below but are **not** blockers, per the operator on 2026-09-07:

- **Rigid-body list is not ours yet.** Motive showed `cf1, cf10, cf11, cf12,
  cf13`, with `cf2`/`cf3` absent, and positions 1.7–3.0 m from the yaml
  `initial_position` values. **A colleague was mid-session with his own Motive
  configuration**; this is not the show layout and nothing should be inferred
  from it about the fleet. Re-check with `python3 scripts/sync_initial_positions.py --dry-run` (workspace root: lists every streamed body against the enabled drones) or the mission console's health graph once the show configuration is actually loaded.
  In particular this run does **not** settle the `cf12` vs `cf14` question
  (§4c); it was someone else's setup.
- **`/poses` ran at ~27 Hz**, drifting down from 31.3, `max` gap 0.215 s, std
  dev 0.04 s — against ~42 Hz measured at the socket. Deliberately parked as
  non-critical for now; see loose end 11 before flying.

#### Verified rig state, 2026-09-07 16:04

Motive streams **multicast** `239.255.42.99:1511` from `192.168.9.124`, and
`scripts/mocap_diag.sh` received it at ~42 Hz pinned to this laptop's
`192.168.9.108` (`wlp131s0f0`, SSID `motive`, mac `24:eb:16:37:e8:da`). The
network path is fine; only the join interface was wrong.

| When (2026-09-04) | Interface | Network | Address |
|---|---|---|---|
| 16:04, 17:44 | `wlp131s0f0` | SSID `motive` | 192.168.9.196 |
| 18:07 onward, and 09-07 | `wlp131s0f0` | SSID `motive` | 192.168.9.108 |
| 17:51–18:07 | `enp130s0` | wired | 192.168.9.109 |
| throughout | `wlp131s0f0` | `AI.R STC Hangar` / `-5G` | 192.168.8.196 (no Motive) |

#### The scripts

- `scripts/mocap_diag.sh` — network-level: addresses+MACs, routing, NatNet
  command socket, and a real multicast join per interface. Needs no drones, no
  radio, no sudo. Safe alongside others sharing the Motive server; it skips the
  port-binding sections if something already holds `:1511`.
- ~~`scripts/mocap_verify.sh`~~ — **removed 2026-09-18.** It ran the apt mocap
  node, which this rig no longer has, so it aborted at its first check. It exposed
  the original root cause; today the same questions are answered by
  `ros2 topic hz /poses` and `python3 scripts/sync_initial_positions.py --dry-run` (workspace root: lists every streamed body against the enabled drones) or the mission console's health graph.

**Why the launch logs could never settle this.** The node's stdout is
block-buffered into the launch pipe and the process is always killed by a
signal, so the buffer never flushes — not even `logClouds=0`. Run it directly or
under `stdbuf -oL`, which both scripts do.

### 4b. The apt-package shadowing trap

`ros-humble-crazyflie` is installed system-wide, and apt provides **every**
package this stack uses — `crazyflie`, `crazyflie-py`, `crazyflie-interfaces`,
`crazyflie-examples`, `crazyflie-sim`, `crazyflie-description`, all at 1.0.5.

If the workspace overlay is not sourced, `ros2 launch crazyflie launch.py`
**silently** resolves to `/opt/ros/humble/` and reads *its* config
(`hostname: 141.23.110.143`). Your edits appear to do nothing. The launch
starts normally, so it reads as a mocap/network fault when it is really the
wrong config file. Confirmed on 2026-09-04: the 16:07 and 16:09 launches ran
`/opt/ros/humble/lib/crazyflie/crazyflie_server`; from 16:12 on they ran the
workspace binary.

```bash
ros2 pkg prefix crazyflie                                  # before every session
grep -o "cmd '[^ ]*crazyflie_server" ~/.ros/log/<run>/launch.log   # audit a past run
```

This is also why a stripped-down workspace is dangerous: a lone show package
builds and runs fine against the apt versions, losing every rig customisation
without a single error.

**One exception, worth knowing.** `motion_capture_tracking` is *only* installed
from apt — it is not built in the workspace, and `install/` contains no copy of
it. The mocap node therefore always runs from `/opt/ros/humble/lib/` no matter
what is sourced, in working runs too. Seeing that path in a launch log is
normal and is **not** a symptom of the shadowing trap; only the
`crazyflie_server` path tells you which config was read.

### 4c. Documentation staleness in the main repo

`CLAUDE.md` is wrong on three points, all still unfixed:

| Says | Actually |
|---|---|
| fleet is `cf14` | yaml has **`cf12`** |
| `origin` = jeremyCHH | `origin` = **AI-DA-STC/CrazySwarm2-with-Mocap** |
| — | `pose_bridge.py` `DRONES` lists `cf1, cf2`; fleet is five |

Confirm cf12-vs-cf14 physically before editing the doc — guessing is how the
discrepancy started.

### 4d. `crazyflie_sim` vs the locally built `cffirmware` — FIXED 2026-09-07

**Symptom:** the sim server dies, mid-show, the first time any script calls
`startTrajectory`. Everything before that works — connect, upload, takeoff,
`goTo` — so it reads as a problem with the trajectory you just uploaded.

```
File ".../crazyflie_sim/crazyflie_sil.py", line 187, in startTrajectory
    firm.plan_start_trajectory(
TypeError: plan_start_trajectory() takes 5 positional arguments but 7 were given
[ERROR] [crazyflie_server-1]: process has died
```

**Cause:** two generations of the cffirmware binding disagree on that call.
`crazyflie_sil.py` was written for the newer one, which splits `relative` into
`relative_position` / `relative_yaw` and also wants the current yaw. The
binding actually installed here is the older five-argument form, built from
`~/crazyflie-firmware`:

```
$ grep -n "def plan_start_trajectory" ~/crazyflie-firmware/build/cffirmware.py
662:def plan_start_trajectory(p, trajectory, reversed, relative, start_from):
```

which matches `src/modules/interface/planner.h:121` in that same tree.

**Fix applied** to `src/crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py`:
the SWIG wrapper is a plain Python function, so its arity is introspectable —
read it once at import and pick the matching call. Works with either binding,
so it survives a firmware update. Revert with

```bash
git -C ~/CrazySwarm2-with-Mocap checkout \
    src/crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py
```

Two things worth carrying forward:

- **This affects every script, not just the shows.** `demo_show` calls
  `startTrajectory` too and would have died at exactly the same point. It is
  the most likely reason nothing had ever flown in sim.
- **Hardware is unaffected.** The C++ `crazyflie_server` talks to the real
  firmware over the radio and never touches this Python binding. The bug is
  `backend:=sim` only.

---

## 5. `reference/` — firmware TOC dumps

`firmware_log_toc.csv` (541 entries) and `firmware_param_toc.csv` (321) are the
log and parameter tables cached by cflib at connect on 2026-09-04. They are the
authoritative answer to *"does this variable exist in the firmware?"* — which
matters because **`crazyflies.yaml`'s `firmware_logging.custom_topics` vars must
exist in the log TOC or the C++ server aborts at connect**, and a log block is
capped at 26 bytes.

All eight `kalman_preflight` vars verified present against this TOC on
2026-09-07:

```
kalman.stateX/Y/Z   motion.deltaX/deltaY   range.zrange   stateEstimateZ.vx/vy
```

Check a candidate var before adding it to a custom topic:

```bash
grep ",<group>,<name>$" reference/firmware_log_toc.csv
```

---

## 6. Safety rules — do not skip

- **Scan every enabled address before every launch.** The C++ server connects
  drones in lexicographic order and **hangs silently forever** on the first
  enabled drone that does not answer. One unreachable drone kills the whole
  launch — no error, no `/all/*` services, needs SIGKILL.
  ```bash
  ros2 run crazyflie scan --address 0xE7E7E7E701
  ```
- **`cf6` is dead on radio** (2026-08-04, silent on full channel/datarate
  sweeps at its own *and* the factory address). Do not re-enable until a scan
  answers.
- **No mocap = fly-away.** `locSrv.extPosStdDev=1e-3` force-fuses mocap
  position, so a yaw offset is invisible at rest and a fly-away in flight. Use
  the preflight GUI's `err.yaw`: ±5° fly, 5–15° fix, >20° no fly.
- **Before blaming the network**, rule out the UDP 1511 orphan:
  ```bash
  ss -uanp | grep :1511    # two sockets = a leftover process is starving mocap
  ```
- `initial_position` comes from `/poses`, never `/cfX/pose`. Each drone at
  **its own** yaml position, ≥1 m apart. Wrong-corner placement produced the
  real collision of 2026-08-04.
- **The firmware has no collision avoidance enabled here.** Crossing goTo paths
  are a real collision. Every simultaneous leg goes through
  `safety.check_leg()` before it flies.

---

## 7. History — how this package came to exist

`~/near-intern/CrazySwarm2-with-Mocap` was a `.git`-stripped copy of the main
repo, taken at `8b52b3c` plus the 2026-09-04 flight session's uncommitted
changes. Its entire delta from the clean repo was five files: `HANDOVER.md`,
the two modified yamls, and the two TOC dumps. All five are preserved here —
the yamls as `config/`, the dumps as `reference/`, the handover as this file.

**That copy was deleted on 2026-09-07.** Everything else in it was byte-
identical to `8b52b3c` and is recoverable from `~/CrazySwarm2-with-Mocap`.

Lesson worth keeping: the copy was made to get a scratch space for rig-state
edits, but deleting `.git` threw away diff, history, and undo — the exact tools
that make such edits safe. **A branch would have been strictly better.** This
directory is not a git repo either; `git init` it before it accumulates
anything irreplaceable.

---

## 8. Loose ends

1. **Add `141.23.110.162/32` to the motive NIC** — the actual unblock (§4a):
   `sudo ip addr add 141.23.110.162/32 dev wlp131s0f0`. **Per-session, by
   choice** — re-apply after every reconnect. Nothing else here matters until
   this is done.
2. ~~Confirm `/poses` publishes~~ — **done 2026-09-07 16:28**, ~27 Hz.
   Still to do **once the show's own Motive configuration is loaded**: re-run
   `python3 scripts/sync_initial_positions.py --dry-run` (workspace root: lists every streamed body against the enabled drones) or the mission console's health graph. The 09-07 list was a
   colleague's transient setup and tells us nothing about the show fleet.
3. ~~`interface_ip`~~ — **inert in this build**; do not spend more time on it.
   Fixing it properly means building `motion_capture_tracking` from source.
4. **Confirm cf12 vs cf14** physically, then fix `CLAUDE.md`.
5. Fix `CLAUDE.md`'s `origin` remote (AI-DA-STC, not jeremyCHH).
6. ~~Re-enable drones~~ — **done 2026-09-07**: cf1, cf2, cf3, cf10, cf12.
   Still **scan each address before every launch** (§6).
7. ~~Fly in sim~~ — **done 2026-09-07**: `swarm_show` runs end to end on
   `backend:=sim`, after fixing the `crazyflie_sim` binding bug in §4d.
   **Still to do: fly on hardware.** Nothing in this package has ever been in
   the air. `SHOW_GUIDE.md` §5d is the pre-flight checklist.
8. `git init` this working directory.
9. Identify the SIGSEGV seen 16:26–17:16 on 09-04 (§4a table). Not reproduced.
10. Consider whether five drones on one `radio://0/80/2M` dongle is enough
    bandwidth, or whether a second dongle/channel is wanted.
11. **`arena_radius` and `ceiling` are unmeasured.** `ShowConfig` defaults to
    2.5 m and 2.0 m, inherited from `safety.py`, and neither has ever been
    checked against the actual mocap volume. They are the only guessed values
    in the package, and they are exactly the check that would stop the show
    flying into a wall. Measure them — `SHOW_GUIDE.md` §2c.
12. **Trajectory upload time on hardware is unmeasured.** The show uploads 5
    figures to 5 drones = 25 blocking round trips over one radio before
    anything arms. Instant in sim; unknown on the real link. If it is slow,
    the lever is fewer figures, not fewer pieces.
13. **Check the mocap frame rate before trusting flight** (deprioritised by the
    operator 2026-09-07 — "might just be something else", not critical yet, but
    unresolved and it belongs in the pre-flight checks).
    Measured: `/poses` ~27 Hz vs ~42 Hz at the socket, max gap 0.215 s. `mocap_diag.sh`
    measured ~42 Hz with ~3.6 kB datagrams — above the 1500-byte MTU, so every
    frame is IP-fragmented and losing one fragment loses the frame. Wi-Fi
    multicast is unacknowledged, which is where that loss comes from. If Motive
    is set to 120/240 Hz then most frames are being dropped, and
    `locSrv.extPosStdDev=1e-3` force-fuses whatever arrives (§6). Check Motive's
    rate, trim the streamed marker set, and try the **wired** NIC before flying
    five drones. Step 2 gives the real `/poses` number.
