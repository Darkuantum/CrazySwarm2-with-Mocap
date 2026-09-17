# CLAUDE.md — CrazySwarm2 (self-contained workspace)

Guidance for Claude Code working in this repository. Read before editing or debugging.

## What this repo is

A **self-contained ROS 2 workspace** for an indoor **Crazyflie 2.1 swarm** with
**OptiTrack** mocap, built on [crazyswarm2](https://github.com/IMRCLab/crazyswarm2).
The full customized source is **vendored in `src/`** (committed), so a clone is a
byte-for-byte copy of the rig. `setup.sh` only installs deps and builds — it does
**not** fetch upstream.

- **`src/` IS committed** — edit source/config here directly; a clone reproduces the rig.
- **`build/`, `install/`, `log/`, `core.*` are git-ignored** — never commit them.

## Architecture (data flow)

```
Motive/OptiTrack ──NatNet Multicast@50Hz──► motion_capture_tracking ──/poses──►
  crazyflie_server ──Crazyradio #0 (radio://0/80/2M)──► cf1 cf2 cf3 cf5 cf8
                    ▲ user scripts via crazyflie_py
                      (cf6 = DEAD on radio 2026-08-04, no longer in the yaml at all)
Alt mocap path: Motive → natnet_ros2 → /<body>/pose → pose_bridge.py → /poses
```

Fleet = **cf1 + cf2 + cf3 + cf5 + cf8 enabled** (five drones), all on **ONE
Crazyradio dongle** (`radio://0/80/2M`, addresses `0xE7E7E7E701/02/03/05/08`).
**`crazyflies.yaml` is the ground truth for the fleet — re-read it rather than
trusting this paragraph**; the roster was last re-flown in `9e23d8a` (2026-09-10),
which dropped cf10/cf12 and brought up cf4/cf8; cf4 was then swapped back to
cf3 (2026-09-16) — same airframe slot/initial_position, address
`0xE7E7E7E703`, so the Motive rigid body must be renamed to `cf3` too. (Note cf1's URI uses the
`radio://*/…` wildcard dongle while the rest pin `radio://0/…`.) `cf6` is dead on
radio 2026-08-04 (silent on full channel/datarate sweeps at its own AND the
factory address; needs a physical check; do NOT re-add until
`scan --address 0xE7E7E7E706` answers) and is no longer in the yaml.
The two-dongle cf1+cf11 setup is
historical; its channel/datarate rules (Gotchas below, `crazyflies.yaml`
comments) still apply **when running two dongles** — read them before editing URIs.

`motion_capture_tracking` (started by `launch.py`) connects directly to Motive —
the natnet_ros2 + `pose_bridge.py` path is an alternative, not the default.
`launch.py` also auto-starts RViz and the **preflight GUI**
(`preflight_kalman_plotter.py` — per-drone go/no-go checks; docs/RUNNING.md Section C).

## Repo layout

```
src/                # VENDORED source (committed)
  crazyswarm2/        # customized: configs, launch.py (foxglove node), scripts, examples
  natnet_ros2/        # OptiTrack driver (+ vendored NatNetSDK)
  motion_capture_tracking/  # VENDORED mocap driver: IMRCLab ros2@64d3af2 + NatNet-4.2 modeldef patch.
                            # NEVER apt-install it: apt 1.0.9 hard-codes IP 141.23.110.162 → no /poses (VENDORED.md)
                            # OptiTrack is the ONLY backend built — Qualisys/Vicon/VRPN/FZMotion are
                            # OFF in its CMakeLists.txt (see the compiler-warning gotcha below)
scripts/
  setup.sh            # install_deps + build (source already present)
  install_deps.sh     # distro-aware apt + rosdep + pip
  build.sh            # colcon wrapper (LOW_MEM=1 for SBCs)
  setup_sim_firmware.sh  # build cffirmware bindings (SIM only)
  led.sh              # set the Color LED deck via `ros2 param set` (server must be running)
  color_led_cflib.py  # LED test straight over cflib (STOP the server first — one radio owner)
  sync_initial_positions.py  # rewrite crazyflies.yaml initial_position from live /poses
pose_bridge.py      # natnet → /poses (NamedPoseArray @ 50 Hz)
console/            # OPTIONAL mission-console GUI (its own README). Self-contained:
                    #   not a colcon package, nothing in src/ imports it, no build.
                    #   `rm -rf console/` removes the feature and changes nothing else.
docs/               # RUNNING, MOCAP, TROUBLESHOOTING
README.md           # single setup doc (no separate SETUP.md)
```

Key customized files inside `src/`:
- `src/crazyswarm2/crazyflie/config/*.yaml` — drone/mocap/server/teleop config.
  `crazyflies.yaml` has the `kalman_preflight` custom log topic (feeds the
  preflight GUI; 22 B of the 26 B log-block budget — vars must exist in the
  firmware log TOC or the cpp server aborts at connect).
- `src/crazyswarm2/crazyflie/launch/launch.py` — adds the **foxglove_bridge** and
  **preflight GUI** nodes; defaults: `rviz` `True`, `preflight` `True`,
  `foxglove` `True`, `gui` `False` (upstream lacks the extra nodes).
- `src/crazyswarm2/crazyflie/scripts/preflight_kalman_plotter.py` — preflight GUI.
  Constants at the top: `ORIENT_WARN_DEG` 10°, `ERR_STALE_S` 3 s, takeoff/land
  setpoints (0.5 m/3 s, 0.03 m/3 s), `LOG_DIR` = `~/crazyswarm_ws/preflight_logs`
  (hardcoded, NOT under this repo). Keys: `r` reset kalman, `e` broadcast e-stop
  (deliberate); takeoff/land have no keys on purpose.
- `src/crazyswarm2/crazyflie/src/crazyflie_server.cpp` — runtime firmware-param
  pushes use `add_on_set_parameters_callback` instead of upstream's
  ParameterEventHandler (self-generated `/parameter_events` never loop back on
  this rig — see Gotchas), so `ros2 param set` now actually reaches the drones;
  also sets the Color LED deck **green on connect** / off on clean disconnect.
- `src/crazyswarm2/crazyflie/config/server.yaml` —
  `query_all_values_on_connect: True` (all firmware-param values are fetched at
  connect, not lazily).
- `src/crazyswarm2/crazyflie_examples/crazyflie_examples/multi_trajectory.py` —
  arms before takeoff; flies ONLY `traj1.csv` (~25 s) on all drones (traj0
  dropped, ~50 s total); return-home goTo (+0.75 m over each drone's own
  `initial_position`) then slow 4 s land (`targetHeight` 0.04).
- `src/crazyswarm2/crazyflie_examples/crazyflie_examples/multi_trajectory_formation.py`
  — entry point in `setup.cfg`: NO traj1 anymore (plain `multi_trajectory`
  still flies it, unchanged) — instead a **formation waypoint tour**
  (3 waypoints + return: WAYPOINT_OFFSETS (0.6,0)/(−0.6,0.5)/(0,−0.6) then
  (0,0), each applied rigidly to EVERY drone's own start hover position, so
  separation stays the 1.36 m start spacing; max excursion ~2.14 m from
  ROOM_CENTER, inside the orbit clearance), then regular **n-gon gather**
  (pentagon at R=0.8; phase offset auto-optimized per initial positions, 1°
  sweep + min-distance assignment) → **ONE continuous smooth +360° rotation**
  (uploaded circle trajectory id 1, 10 s — NOT stepped goTos) →
  **triangle+tail-pair morph** (5 slots) → rigid **swarm ORBIT** around
  ROOM_CENTER (0.0467, −0.1037) at R=1.2 (shared circle trajectory id 2, 12 s,
  ~0.63 m/s tangential / ~0.33 m/s² centripetal, `relative=True` = rigid
  translation) → **expand back onto the gather pentagon** → **home**
  (a DIRECT return crossed paths at R=2.0 — 1 crossing verified — the
  pentagon intermediate step is kept at R=1.2 even though the direct
  return is crossing-free there) + slow
  4.5 s land (~0.16 m/s descent). Long moves (waypoint legs, ring shift,
  both return legs) use distance-scaled goTo durations
  `max(2.0, longest_xy/0.5)` s (≤0.5 m/s avg); short morphs
  keep TRANS_DURATION=2.0. Min separation 0.84 m; **orbit sweeps ~2.24 m
  radius around ROOM_CENTER (keep clear — shrank from ~3.05 m at R=2.0)**;
  ≈58.0 s takeoff→landed; formation altitude 1.0 m
  (FORM_HEIGHT). Trajectory
  memory 12 pieces (6 rotation circle at pieceOffset 0 + 6 orbit at offset
  6; traj1's 16 dropped) ≈1.6 KB of the firmware's
  ~4 KB. docs/RUNNING.md Section B.
- `src/crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py` —
  `plan_start_trajectory` is called through an **arity probe**
  (`_PLAN_START_TRAJECTORY_NARGS`, `inspect.signature` at import) so both the
  5-arg and 7-arg cffirmware bindings work (see Gotchas).

## Build & run

```bash
# ROS 2 must be installed first (manual; see README Setup Step 1).
./scripts/setup.sh                                  # full setup + build
./scripts/setup_sim_firmware.sh                     # only if using backend:=sim
source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
ros2 launch crazyflie launch.py backend:=sim        # rviz + preflight GUI on by default
ros2 run crazyflie_examples hello_world             # takeoff/hover/land
```
Supported: **Ubuntu 22.04 + Humble** and **24.04 + Jazzy** (auto-detected from
`/etc/os-release`). Tested by running on Jazzy; Humble is verified by inspection.

## Gotchas (hard-won — don't re-derive)

- **`set -u` vs ROS `setup.bash`.** Sourcing `/opt/ros/<distro>/setup.bash` under
  `set -u` aborts on the unbound `AMENT_TRACE_SETUP_FILES`. `build.sh` wraps the
  source in `set +u`/`set -u`. Without it the build silently never runs → empty
  `install/`.
- **cffirmware (simulator only).** `crazyflie_sim` imports `cffirmware` (Crazyflie
  firmware Python bindings) — not a pip package. `setup_sim_firmware.sh` builds it
  from `crazyflie-firmware` (tag 2025.02). NOT needed for hardware backends.
  - Its **CMSIS submodule needs `git-lfs`** or the checkout aborts (`arm_add_f32.c`
    missing). The script installs git-lfs.
  - The `setup.py` egg does **not** bundle the compiled `_cffirmware*.so`; expose
    `crazyflie-firmware/build` on `PYTHONPATH` instead (script appends to `~/.bashrc`).
- **conda shadows system Python.** ROS 2 runs nodes with `/usr/bin/python3`. A conda
  base env (different Python) causes `No module named '_cffirmware'` and rclpy errors.
  Deactivate conda for ROS work. `build.sh` now defends the BUILD only: if
  `CONDA_PREFIX`/`VIRTUAL_ENV` is set or `python3` isn't `/usr/bin/python3` it strips
  conda/venv from `PATH`+`PYTHONPATH` and passes `-DPython3_EXECUTABLE=/usr/bin/python3`.
  Without that, rosidl generated the Python message bindings for the wrong interpreter
  (`cpython-312` .so on a 22.04 box whose `ros2` runs 3.10) — C++ nodes fine, but every
  `ros2 topic echo` of a workspace message died with
  `The message type '.../NamedPoseArray' is invalid`. RUNTIME is still undefended:
  deactivate conda before `ros2 launch`.
- **Preflight GUI needs `libxcb-cursor0` on 22.04/jammy.** pip `cfclient` pulls
  PyQt6 >= 6.5, whose xcb platform plugin needs it; jammy doesn't install it by
  default, so the preflight node dies at start with
  `Could not load the Qt platform plugin xcb`. `install_deps.sh` installs it
  (plus `python3-tk`).
- **Compiler warnings: first-party code must stay clean; ~25 remain, all
  third-party and reviewed.** A clean build used to emit ~77 warnings. Most came
  from mocap backends this rig never uses, so they are no longer compiled at all
  (see `motion_capture_tracking/CMakeLists.txt`) — that removed an overlapping-buffer
  `sprintf` in the Qualisys SDK, strict-aliasing type-punning, and a truncating
  `strncpy` in VRPN, none of it reachable with `type: "optitrack"`. The ~25 left are
  ALL in `deps/librigidbodytracker` (+1 in `deps/libmotioncapture/src/optitrack.cpp`)
  and were each checked: `-Wsign-compare` on loops bounded by drone counts,
  `-Wreorder` whose initializers are all constants (so declaration-order init is
  identical), and unused locals. **Benign — do not mass-patch upstream to silence
  them, and do NOT add `-w`/`-Wno-*` to those targets**: this workspace patches
  vendored deps (e.g. the NatNet timeout in `optitrack.cpp`), so blanket suppression
  would hide OUR future mistakes in exactly the files we edit. Any NEW warning in
  `src/natnet_ros2` or `src/crazyswarm2` is a real regression — fix it.
- **`declare_parameter(name, {})` is a trap.** A bare `{}` is ambiguous between the
  `(name, default_value, …)` and `(name, ParameterDescriptor, …)` overloads; gcc
  picks the descriptor one, declaring the parameter with NO default so rclcpp throws
  `NoParameterOverrideProvided` at runtime instead of using the empty default. It
  warns at compile time but only as "would use explicit constructor". Always spell
  the default out (`std::vector<std::string>{}`). Fixed in `natnet_ros2.cpp`.
- **RViz and the preflight GUI are ON by default** in `launch.py`
  (`rviz:=false` / `preflight:=False` to disable). `foxglove:=True` by default
  but needs `ros-$ROS_DISTRO-foxglove-bridge` (installed by `install_deps.sh`);
  view via the Foxglove Studio app, not a window.
- **Mocap "died" / `/poses` 0 publishers** — CONFIRMED cause on this rig: a
  leftover process bound to UDP 1511. Two `SO_REUSEPORT` sockets on 1511 → the
  kernel gives each NatNet datagram to only ONE socket → the mocap node starves
  and hangs silently in libmotioncapture `connect()` (no crash, not in
  `ros2 node list`). Diagnose `ss -uanp | grep :1511`; kill the orphan and
  relaunch. Ping to the Motive PC proves nothing (unicast ≠ multicast).
- **Motive address:** `motion_capture.yaml` `hostname: "auto"` → `launch.py`
  discovers the Motive PC via a NatNet ping broadcast on UDP 1510 (lab DHCP
  drifts: .100 → .124 → .152). Override: `mocap_hostname:=<ip>` launch arg or
  `CRAZYSWARM_MOCAP_HOST`. Must be an IPv4 literal; the launch aborts loudly if
  nothing answers instead of the node hanging silently.
- **Motive transmission type is read once at connect** (the vendored
  `motion_capture_tracking` requires Multicast or Broadcast Frame Data; never
  install the apt package — 1.0.9 hard-codes a foreign interface IP) — after changing it, fully
  restart the launch. A frozen mocap node ignores SIGINT (blocked in `recv`);
  launch escalates to SIGKILL — check `pgrep -f motion_capture_tracking` for
  leftovers (they make the next connect SIGABRT).
- **Rigid-body orientation.** Create the Motive rigid body with the drone's
  forward axis on global +X. `locSrv.extPosStdDev=1e-3` force-fuses mocap
  position, so position error is ~1 mm even with a rotated body — a yaw offset
  is invisible at rest and a fly-away in flight. The preflight GUI's banner /
  `err.yaw` catches it (±5° fly; 5–15° fix; >20° no fly).
- **ROS apt 404 churn.** `ros-<distro>-{sensor-msgs,tf2-ros,ament-cmake-auto}` can
  404 when apt tries to *upgrade* to a pruned pool version. `install_deps.sh` uses
  `--no-upgrade` for these (desktop already provides them).
- **natnet NatNet SDK** is vendored under `src/natnet_ros2/deps/NatNetSDK/`
  (x86_64). On a different arch the build re-downloads it via `wget` (needs internet).
- **Same-process DDS loopback failure.** On this rig a node's OWN
  `/parameter_events` messages are never delivered back to itself (events from
  *other* processes arrive fine). Three consequences: (1) runtime firmware-param
  pushes in `crazyflie_server.cpp` use `add_on_set_parameters_callback`
  (synchronous, inside the `set_parameters` service call — no pub/sub
  round-trip) instead of upstream's ParameterEventHandler, which never fired,
  so `ros2 param set` silently never reached the drones; (2) `scripts/led.sh`
  deliberately drives the `ros2` CLI (its long-running daemon keeps the ROS
  graph warm) instead of rclpy — fresh rclpy processes see DDS discovery never
  complete and parameter service calls time out; (3) `Crazyswarm()` in a fresh
  rclpy process can hang waiting on `all/emergency` for the same reason
  (`color_led.py` avoids Crazyswarm() and calls `set_parameters` directly).
- **Server BLOCKS FOREVER on the first unreachable enabled drone.** The cpp
  server connects drones in lexicographic `std::map` order and hangs
  **silently** on the first enabled drone that doesn't answer radio — one
  unreachable drone kills the whole launch (no error, no `/all/*` services,
  needs SIGKILL). Go/no-go rule: **scan every enabled address before every
  launch** (currently `0xE7E7E7E701/02/04/05/08` — confirm against the yaml).
  cf6 died this way 2026-08-04 (silent on full channel/datarate
  sweeps at its own AND factory address — physical check needed).
- **Two Crazyradios (when running two dongles — current rig is single-dongle):
  channels must be ≥2 apart at 2M.** A 2M channel is ~2 MHz wide, so adjacent
  channels overlap and crazyflie-link-cpp refuses the pair — USBManager.cpp:
  `"Channels 80 and 81 are already served by Crazyradio 0"`. Historical
  two-dongle rig: cf1 on `radio://0/80/2M`, cf11 on `radio://1/90/1M`
  (10 channels clear).
- **URI datarate must match the drone.** A wrong datarate in the URI (e.g. `2M`
  for a drone talking `1M`) makes the server hang **forever** waiting for a
  drone that never answers — no error, and the `/all/*` services are never
  created. Verify with a scan on the drone's address
  (`scan --address 0xE7E7E7E711` → `radio://*/90/1M/...`).
- **`initial_position` comes from `/poses`, never `/cfX/pose`.** The onboard
  estimate is seeded by the yaml — copying it back is circular. **Use
  `scripts/sync_initial_positions.py`** (mocap up, server not yet started): it
  samples `/poses`, matches each ENABLED drone to the rigid body of the same
  name, and rewrites only the `initial_position` triples in place (comments
  preserved). It REFUSES to write when a drone is not streamed, is moving
  (>10 mm spread over the window), sits above 0.5 m, or when two drones are
  <1 m apart — `--dry-run` to preview, `--force` to override, `--with-z` to
  write the measured z instead of keeping the yaml's. Manual equivalent: read
  x/y from `ros2 topic echo /poses | grep -A5 -- '- name: cf1$'` and edit by
  hand. Either way, restart the server (yaml read only at launch). Enabled
  drones **≥1 m apart** (same-trajectory flight preserves start separation),
  and each drone at ITS OWN yaml position — wrong-corner placement = crossing
  goTo paths = the real collision of 2026-08-04.
- **Sim `plan_start_trajectory` arity — DO NOT ASSUME, PROBE.** This entry used
  to claim the binding is 7-arg (`relative_position`/`relative_yaw` +
  `start_from`/`start_yaw`). That is **backwards** for the build on this box:
  verified 2026-09-17, `~/crazyflie-firmware/build/cffirmware.py` exposes
  `plan_start_trajectory(p, trajectory, reversed, relative, start_from)` —
  **5 args**. Calling it with 7 raises `TypeError: takes 5 positional arguments
  but 7 were given`, which **kills the sim server outright at the first
  `startTrajectory`** — the demo runs fine through takeoff, waypoints and the
  gather, then the server dies mid-flight and the script hangs forever in the
  next `sleep`. `crazyflie_sim/crazyflie_sil.py` now picks the call by
  `inspect.signature(...)` arity at import, so either binding works. Sim-only;
  hardware unaffected.
- **Sim runs need `--ros-args -p use_sim_time:=true`** for the trajectory
  demos (`multi_trajectory`, `multi_trajectory_formation`) — the sim clock is
  ~4x slower than wall time, so without the flag the script races ahead of the
  physics. Hardware runs WITHOUT the flag.
- **LED convention: green = drone connected and ready for command; red =
  drones are in flight / a script is controlling them.** The server turns the
  Color LED deck green on connect (and off on clean disconnect); every
  crazyflie_py script sets it red for its lifetime and restores green on exit
  (`crazyswarm_py.py`, atexit — survives exceptions and Ctrl-C; if rclpy is
  already shut down the deck keeps its last color). Change colors manually
  with `scripts/led.sh` or `ros2 run crazyflie_examples color_led`.

## The `console/` module (optional GUI)

A browser front-end for the rig — `./console/run.sh` → http://localhost:8077 —
covering the launch, the preflight scans/checks, the flight and service commands,
live YAML editing, and a system-health diagram that locates the failing link
instead of leaving it in the launch log. Read `console/README.md` before touching
it. Four rules hold it together; keep them when editing:

- **Stay decoupled.** It is deliberately removable: don't make `launch.py`,
  `CMakeLists.txt`, `setup.sh` or anything in `src/` depend on it, and don't turn
  it into a colcon package. The only links pointing at it are prose (README
  section 8, this file).
- **`ros2` CLI, never in-process rclpy.** Every probe and action is a subprocess
  running the same command a human would type. Two reasons: a fresh rclpy process
  on this rig can stall forever in DDS discovery (same reason `scripts/led.sh`
  uses the CLI), and the user's stated goal is to *learn* the commands from the
  GUI — so every button and every health check displays its exact argv, fetched
  from the backend that will execute it.
- **YAML writes are text-surgical** (`configio.YamlText`), never `yaml.dump()`:
  these config files carry the hard-won comments, and a round-trip deletes them.
  Writes are parse-checked, diffed and backed up to `console/backups/`.
- **`health.py` is where the Gotchas above become executable.** Each known silent
  failure is a graph node (UDP 1511 starvation, apt mocap driver shadowing, server
  blocked mid-connect with no `/all/*`, datarate mismatch, conda python). Adding a
  gotcha to this file is only half the job — add the probe there too, as a node +
  edge, or as a `SIGNATURES` regex that attaches the log line to an existing node.

## Conventions for Claude

- **Edit source/config directly in `src/`** and commit — there is no overlay or
  re-import that would clobber it. A clone reproduces exactly what's committed.
- After editing `src/`, rebuild with `./scripts/build.sh` (or `build.sh <pkg>`),
  then re-source `install/setup.bash`. Rebuild dependents after `.msg`/`.srv` edits.
- `pose_bridge.py` `DRONES` and `PUBLISH_HZ` must match `crazyflies.yaml` and the
  Motive streaming rate (50 Hz). Currently STALE: it lists `cf1, cf2` but the
  enabled fleet is `cf1, cf2, cf3, cf10, cf14` — fix `DRONES` before using the
  alt mocap path.
- Keep scripts distro-parameterized (`ros-${ROS_DISTRO}-…`); never hardcode `jazzy`.
- **NEVER push, merge or force anything to `upstream` (AI-DA-STC). FETCH ONLY.**
  This is the shared team repo; changes reach it by **Pull Request from the
  `Darkuantum` fork**, reviewed by a human — never by a direct push. The danger is
  real, not theoretical: **another git user on this laptop holds credentials that
  CAN push to AI-DA-STC**, so a stray `git push upstream` may silently succeed
  instead of being rejected, landing unreviewed code straight on the team's `main`.
  Guard in place: `upstream`'s push URL is set to the bogus
  `DISABLED-open-a-PR-instead`, so `git push upstream …` dies locally before it
  touches the network or any credential. **Do not "fix" that push URL** — it is
  deliberate. It does NOT cover every route, so also never run:
  `git push <any AI-DA-STC URL>`, `git push --all/--mirror` (iterates remotes),
  or any push while a different `user`/credential helper is active. Fetching is
  fine and expected: `git fetch upstream`, `git merge upstream/main`,
  `git log upstream/main`.
- `gh` is not installed here and pushes need the user's GitHub auth — don't attempt
  to push; report and let the user push. Remotes: `origin` =
  `git@github.com:Darkuantum/CrazySwarm2-with-Mocap.git` (the ONLY push target —
  SSH, keyed by `core.sshCommand`), `upstream` =
  `https://github.com/AI-DA-STC/CrazySwarm2-with-Mocap.git` (fetch-only; this repo
  is a real GitHub fork of it).
