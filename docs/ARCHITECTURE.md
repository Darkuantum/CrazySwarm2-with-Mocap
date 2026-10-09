# Architecture: what is what in this repo

This is a **self-contained ROS 2 workspace** for an indoor Crazyflie 2.1 swarm flown
under OptiTrack mocap, built on [crazyswarm2](https://github.com/IMRCLab/crazyswarm2).
The full customized source is **vendored in `src/` and committed**, so a clone is a
byte-for-byte copy of the rig — you edit source and config in `src/` directly, there is
no overlay and nothing re-fetches upstream. `build/`, `install/`, `log/` and `core.*`
are git-ignored build output: delete them freely, rebuild with `./scripts/build.sh`.

Two halves below: **what lives where** (file structure), then **what runs and what
flows** (functional graph). `CLAUDE.md` remains the operational ground truth; this file
is the map, not the manual.

---

## Part 1 — File structure

Verified 2026-10-08 against the filesystem (`ls`, `git ls-files`, `git check-ignore -v`,
`find`) and `git diff --stat upstream/main HEAD -- src`. First-party content is listed
file by file; vendored third-party dependencies get one line each.

**Legend.** `[ign]` = git-ignored, never commit. `[CUST]` = vendored upstream code that
WE edit (diff it against `upstream/main` before believing anything about it).
`[VEND]` = vendored, untouched, do not edit. `[1P]` = first-party, written here.
Edited-by: `you` = a human edits it by hand; `tool` = a script or the console rewrites
it; `nobody` = generated or frozen.

```
CrazySwarm2-with-Mocap/
|
|-- CLAUDE.md                 [1P] operational ground truth, the most accurate doc; you + Claude
|-- README.md                 [1P] the single setup doc (there is no SETUP.md); you
|-- WORKSPACE-NOTES.md        [1P] the FORK itself: branch policy, upstream relationship; you
|-- .gitignore .gitattributes [1P] ignore rules (build/ install/ log/ core.* log*.csv ...); you
|
|-- docs/                     [1P] operator documentation
|   |-- ARCHITECTURE.md         this file: file structure + functional graph
|   |-- RUNNING.md              what to run, in order (Sections A/B/C); you
|   |-- MOCAP.md                Motive/OptiTrack setup, axes, rigid bodies; you
|   |-- TROUBLESHOOTING.md      symptom -> cause lookup; you
|   |-- CONSOLE.md              the GUI alternative to typing ros2 commands; you
|   `-- WRITING-A-SHOW.md       how to add your own show on top of crazyflie_shows; you
|
|-- runbooks/                 [1P] per-show OPERATOR runcards (moved out of the show package 2026-10-08)
|   |-- README.md               index of the runcards
|   |-- CAROUSEL.runcard.md     carousel show, flight-day card
|   |-- CONSTELLATION.runcard.md  constellation show, flight-day card
|   `-- ESCORT.runcard.md  ESCORT.narrative.md  ESCORT.qna.md
|                               escort demo: flight card, spoken script, Q&A
|                               (show DESIGN docs stay in src/.../crazyflie_shows/ on purpose —
|                                see "single sources of truth" at the end)
|
|-- scripts/                  [1P] operator tools you run from a terminal, not with `ros2 run`
|   |-- setup.sh                install_deps + build; does NOT fetch upstream
|   |-- install_deps.sh         distro-aware apt + rosdep + pip
|   |-- build.sh                colcon wrapper (LOW_MEM=1 for SBCs; strips conda/venv)
|   |-- setup_sim_firmware.sh   builds the cffirmware bindings, SIM ONLY
|   |-- scan_fleet.sh           go/no-go radio scan of every ENABLED drone; fleet read from the yaml
|   |-- which_server.sh         reports whether a hardware, sim, both or no crazyflie_server is live
|   |-- sync_initial_positions.py  rewrites initial_position in crazyflies.yaml from live /poses  [tool]
|   |-- place_drones.py         placement helper for the floor marks
|   |-- led.sh                  Color LED deck via `ros2 param set` (server must be running)
|   |-- color_led_cflib.py      LED test over cflib (STOP the server first: one radio owner)
|   |-- deck_check.py           which decks are fitted per drone, over cflib (STOP the server first)
|   |-- check_mocap_yaw.py      rigid-body yaw check
|   |-- measure_arena.py        arena survey; feeds the safety.ARENA_* constants
|   |-- arena_flight_sweep.py   flown arena-edge sweep
|   |-- link_stall_recorder.py  dumps the 60 s precursor for EVERY drone the moment one stalls
|   `-- watch_escort.py         escort demo monitor
|
|-- pose_bridge.py            [1P] ALT mocap path: /mocap/<name>/pose -> /poses @ 50 Hz; roster from the yaml
|-- mocap_watchdog.py         [1P] cuts motors / lands if /poses stalls while flying
|-- safety_watchdog.py        [1P] mocap-free runaway watchdog over ROS 2
|-- safety_watchdog_cflib.py  [1P] same idea, standalone, no ROS (owns its own radio)
|-- plot_estimate.py          [1P] live plot of stateEstimate x/y/z/yaw
|                               (these five sit at the root beside scripts/; see "Findings" below)
|
|-- console/                  [1P] OPTIONAL mission-console web GUI — REMOVABLE (`rm -rf console/`
|   |                              changes nothing else). docs/CONSOLE.md is the user guide.
|   |-- README.md               read before editing it
|   |-- run.sh  stop_stack.sh   start/stop; serves http://localhost:8077
|   |-- mission_console/        Python backend: server.py procs.py catalog.py configio.py health.py
|   |   |                                      missions.py usage.py builds.py __main__.py __init__.py
|   |   `-- static/             app.js index.html style.css
|   |       |-- mission/        the MISSION WINDOW (/mission): app.mjs ros.mjs scene.mjs — one
|   |       |                   per demo, rendered from crazyflie_shows/missions/*.yaml (docs/MISSIONS.md)
|   |       `-- vendor/         Preact+htm, three.js — committed, the rig network has no internet
|   |                           (static files are re-read on page load; the BACKEND is not —
|   |                            restart the console after changing Python)
|   |-- tests/                  unittest, no ROS needed: python3 -m unittest discover -s tests
|   |-- backups/              [ign] timestamped yaml copies it writes (console/.gitignore)  [tool]
|   |-- usage/                [ign] usage.jsonl (what the operator runs) + dashboard.json (pins)  [tool]
|   `-- .gitignore
|
|-- src/                      VENDORED + customized workspace source, COMMITTED (a clone is the rig)
|   |
|   |-- crazyswarm2/          [CUST] fork of IMRCLab/crazyswarm2
|   |   |-- crazyflie/          [CUST] the core package: C++ server + launch + config
|   |   |   |-- config/
|   |   |   |   |-- crazyflies.yaml   fleet roster, URIs, initial_position, the kalman_preflight
|   |   |   |   |                     custom log topic (+52 lines vs upstream, git diff --stat); you + tool
|   |   |   |   |-- server.yaml       server params: telemetry watchdog, keepalive_frequency,
|   |   |   |   |                     query_all_values_on_connect, warnings (54 added / 3 removed); you
|   |   |   |   |-- motion_capture.yaml  mocap type / hostname ("auto") / rate / QoS; you
|   |   |   |   |-- teleop.yaml       teleop mapping (stock)
|   |   |   |   |-- aideck_streamer.yaml  stock, unused on this rig
|   |   |   |   `-- config.rviz (+24 lines)  main_config.rviz   RViz layouts
|   |   |   |-- launch/
|   |   |   |   |-- launch.py     [CUST] 26 added / 3 removed: trace_cf and server:= arguments,
|   |   |   |   |                 Motive discovery for hostname "auto" (the foxglove and
|   |   |   |   |                 preflight nodes are ALREADY in upstream, SERVER-CHANGES.md s0)
|   |   |   |   `-- teleop_launch.py  launch_teleop2.py   stock
|   |   |   |-- src/
|   |   |   |   |-- crazyflie_server.cpp  [CUST] 445 added / 22 removed vs upstream (numstat) — the
|   |   |   |   |                         biggest divergence in the repo; the change list is
|   |   |   |   |                         docs/SERVER-CHANGES.md; read it before editing
|   |   |   |   `-- teleop.cpp    stock
|   |   |   |-- scripts/
|   |   |   |   |-- preflight_kalman_plotter.py  [1P] the preflight GUI, auto-started by launch.py
|   |   |   |   `-- aideck_streamer.py cfmult.py chooser.py flash.py gui.py   stock
|   |   |   |-- urdf/           stock robot model
|   |   |   `-- deps/crazyflie_tools/crazyflie_cpp/   [CUST] the radio stack
|   |   |                       edited: include/crazyflie_cpp/Crazyflie.h (266/68),
|   |   |                       src/Crazyflie.cpp (212/9), include/crazyflie_cpp/crtp.h (51/0),
|   |   |                       crazyflie-link-cpp/src/CrazyradioThread.cpp (27/0),
|   |   |                       crazyflie-link-cpp/src/ConnectionImpl.h (16/0), Connection.{h,cpp}
|   |   |
|   |   |-- crazyflie_shows/  [1P] THE SHOW PACKAGE — deliberately kept donatable (designed to
|   |   |   |                      lift out whole, which is why its design docs live inside it)
|   |   |   |-- crazyflie_shows/
|   |   |   |   |-- abort.py preflight.py safety.py   the safety machinery EVERY show must call
|   |   |   |   |-- choreography.py figures.py        show building blocks
|   |   |   |   |-- swarm_show.py demo_show.py plan_show.py            carousel
|   |   |   |   |-- constellation.py constellation_show.py            constellation
|   |   |   |   `-- escort.py escort_show.py escort_teleop.py escort_viz.py plan_escort.py
|   |   |   |                                                         escort demo (reactive)
|   |   |   |-- launch/show_launch.py   show launch (prints the FLEET YAML OVERRIDE banner)
|   |   |   |-- missions/               escort.yaml carousel.yaml constellation.yaml: the console's
|   |   |   |                           MISSION WINDOW specs (docs/MISSIONS.md). Inert data — no show
|   |   |   |                           reads them; they parse the shows' prints, edit them together
|   |   |   |-- scripts/mocap_diag.sh   mocap diagnosis
|   |   |   |-- reference/              firmware log/param TOC csv + two HTML reports; frozen
|   |   |   |-- data/                   empty (.gitkeep)
|   |   |   |-- SHOW_GUIDE.md CONSTELLATION.md ESCORT.md HANDOVER.md PROVENANCE.md
|   |   |   |                           show DESIGN docs; you
|   |   |   `-- setup.cfg package.xml CMakeLists.txt resource/
|   |   |                 entry points: demo_show swarm_show plan_show constellation_show
|   |   |                 plan_constellation escort_show escort_teleop plan_escort
|   |   |
|   |   |-- crazyflie_examples/   [CUST] stock examples; edited: multi_trajectory_formation.py
|   |   |                         (+118), package.xml (+4)
|   |   |-- crazyflie_py/         [CUST] the Python API every show uses (crazyflie.py +14)
|   |   |-- crazyflie_interfaces/ [CUST] msgs/srvs (UploadTrajectory.srv +6: success/message)
|   |   |-- crazyflie_sim/        [CUST] simulator; edited: crazyflie_sil.py (+35, the arity
|   |   |                         probe), crazyflie_server.py (+21)
|   |   |-- crazyflie_server_py/  [VEND] alternative Python server, unused here
|   |   `-- docs/ docs2/ prebuilt/ ros_ws/ systemtests/ rosinstall .github/   [VEND] upstream extras
|   |
|   |-- motion_capture_tracking/  [CUST] vendored mocap driver (IMRCLab ros2@64d3af2 + patches)
|   |   |-- VENDORED.md           provenance — read before touching. NEVER apt-install this
|   |   |                         package: apt 1.0.9 hard-codes a foreign IP and you get no /poses
|   |   |-- patches/              0003 NatNet 4.2 modeldef segfault, 0004 humble tf broadcaster,
|   |   |                        0005 NatNet connect timeout + logging
|   |   |-- motion_capture_tracking/
|   |   |   |-- CMakeLists.txt    [CUST] +19: OptiTrack is the ONLY backend built
|   |   |   |                     (Qualisys/Vicon/VRPN/FZMotion OFF — see the warnings gotcha)
|   |   |   |-- src/motion_capture_tracking_node.cpp   config/cfg.yaml rviz.rviz   launch/
|   |   |   `-- deps/libmotioncapture  deps/librigidbodytracker   [VEND] (one edit: optitrack.cpp)
|   |   `-- motion_capture_tracking_interfaces/   [VEND] msgs, incl. NamedPoseArray
|   |
|   `-- natnet_ros2/            [CUST] ALT mocap driver (+ vendored NatNet SDK)
|       |-- src/natnet_ros2.cpp (+9)   launch/natnet_ros2.launch.py (+11, namespace:=mocap)
|       |-- config/ include/ srv/ scripts/ ui/ natnet_ros2_py/   marker_poses_server.cpp nn_filter.cpp
|       `-- deps/NatNetSDK/     [VEND] x86_64 SDK; its contents are [ign] via src/natnet_ros2/.gitignore
|
|-- data/                     [1P] measurement artefacts — the evidence behind the docs
|   |-- arena/                  climb_cf5.csv verify_cf3.csv (arena survey flights)
|   |-- linkstalls/             stall_cf*.json + link_*.csv written by scripts/link_stall_recorder.py
|   `-- patch-backups/          UNTRACKED leftover: `trace-627b499/...` is EMPTY DIRECTORIES ONLY
|                               (verified `find data/patch-backups -type f` -> 0 files). Safe to
|                               delete; use git to revert traced files, as CLAUDE.md says.
|
|-- Pics/                     [1P] README / preflight / mocap-axis screenshots (tracked .png/.jpg)
|-- video/                    [1P] demo media: .mp4 and .gif tracked; video/*.mov is [ign]
|-- cache/                    TRACKED but probably should not be: two cflib TOC caches
|                             (9B42C0DF.json, E46BE59F.json) the server wrote into the cwd.
|                             Regenerated every run. See "Findings" below.
|-- .claude/                  [1P] Claude Code config: agents/ (build-doctor, config-editor,
|                             mocap-doctor, preflight-analyst), workflows/
|
|-- build/  install/  log/    [ign] colcon output; nobody edits. Rebuild with scripts/build.sh
|-- core.*                    [ign] crash dumps
|-- log*.csv  params*.csv     [ign] cflib TOC caches the server drops in the cwd
`-- __pycache__/ (any depth)  [ign]
```

### The config files that matter

| File (all in `src/crazyswarm2/crazyflie/config/`) | Read by | When |
|---|---|---|
| `crazyflies.yaml` | `launch.py` (`crazyflies_yaml_file`, parsed into the `crazyflie_server` params); also every planner, `scan_fleet.sh`, `pose_bridge.py`, `deck_check.py`, the console | the server reads it **only at launch** — restart after editing; the others read it per run |
| `server.yaml` | `launch.py` merges `/crazyflie_server: ros__parameters` into the same server | at launch |
| `motion_capture.yaml` | `launch.py` (`motion_capture_yaml_file`), which resolves `hostname: auto` and starts `motion_capture_tracking` | at launch; Motive's transmission type is read once **at connect**, so change it and you must fully restart |

Read from `launch.py` (the yaml loads around lines 125–166, the launch arguments around
263–266). `teleop.yaml` is a fourth, used only by the teleop launches.

### Findings from this check (act on them or confirm them)

- `cache/` is **tracked** (`git ls-files cache` returns both files) even though it is
  regenerated cflib output. `.gitignore` covers `log*.csv` and `params*.csv` but not
  `cache/`. Recommend `git rm -r --cached cache/` plus an ignore rule.
- `data/patch-backups/trace-627b499/` contains **zero files** — empty directories only,
  untracked. Nothing to preserve; deleting it matches CLAUDE.md, which already says that
  backup set is gone and that git is the way to revert traced files.
- Root-level `pose_bridge.py`, `mocap_watchdog.py`, `safety_watchdog*.py` and
  `plot_estimate.py` sit beside `scripts/`. Moving them into `scripts/` is a doc-and-link
  change, not a code change — decide before any restructuring, not during.
- The `upstream/main` diff still lists `runcards/` paths because the `git mv` to
  `runbooks/` is staged, not committed; those line counts shift once it lands.

### What this check did NOT verify (stated honestly)

- The vendored deps (`libmotioncapture`, `librigidbodytracker`, `crazyflie-link-cpp`)
  were not read file by file; their `[CUST]` / `[VEND]` marks come from
  `git diff --stat upstream/main HEAD` alone.
- `crazyflie_server_py`, `docs`, `docs2`, `prebuilt`, `ros_ws` and `systemtests` are
  marked `[VEND]` because they are **absent from the diff stat**, not because every file
  was compared.
- Who reads `teleop.yaml` and `aideck_streamer.yaml` is inferred from the launch-file
  names; `aideck_streamer.yaml` usage was not traced at all.
- The readers of `crazyflies.yaml` beyond `launch.py` (`deck_check.py`, `scan_fleet.sh`,
  the console, `pose_bridge.py`) are taken from CLAUDE.md and were not re-verified in
  source here.

---

## Part 2 — Functional graph: what runs and what flows

Evidence basis: topic names, types, QoS and timers below were read from source on
2026-10-08 (`crazyflie_server.cpp`, `launch.py`, `motion_capture_tracking_node.cpp`,
`pose_bridge.py`, `crazyflie_py/crazyflie.py`, `crazyflie_sim/crazyflie_server.py`, the
shows, `console/mission_console/`). Rates marked *cfg* come from a YAML and **will
drift** — print them, never copy them (`./scripts/scan_fleet.sh --list` for the fleet,
`ros2 topic hz` for live rates). Rates marked *meas* are from dated CLAUDE.md entries.

### 1. The diagram (hardware path, the default)

```
  WINDOWS PC                     THIS LAPTOP (one ROS 2 domain; domain 0 is shared with the lab)
  ----------                     ----------------------------------------------------------------
  OptiTrack cameras
        |
     Motive --- NatNet multicast --> [motion_capture_tracking_node]      (VENDORED; never apt)
     (Multicast/Broadcast           |  package motion_capture_tracking, started by launch.py
      frame data, read ONCE         |  owns UDP 1511  (a 2nd socket there = silent starvation)
      at connect; 50 Hz stream)     |  hostname "auto" -> launch.py NatNet ping on UDP 1510
                                    |
                                    +-- /poses  NamedPoseArray  BEST_EFFORT, keep_last(1),
                                    |           deadline from motion_capture.yaml (50 Hz, cfg)
                                    +-- /tf     (child_frame_id per rigid body)
                                    |
      +-----------------------------+------------------------+--------------------+
      |                             |                        |                    |
      v                             v                        v                    v
 [crazyflie_server]           [preflight GUI]          shows / sync script    RViz, Foxglove
 package crazyflie, C++       preflight_kalman_        (read /poses; MUST     (/tf,
 ONE process, ALL drones      plotter.py                use sensor QoS)        robot_description)
      |
      | posesChanged(): per rigid body -> note z for the telemetry watchdog's airborne gate,
      |   then sendExternalPositions / sendExternalPoses on the BROADCAST connection
      |
      | owns: the ONE Crazyradio (radio://0/80/2M — one owner at a time)
      |       callback_group_cf_srv (ONE fleet-wide, mutually exclusive group)
      |
      +===== Crazyradio dongle =====> drones cfN   (CRTP unicast + broadcast)
      ^                                     |
      | services /cfX/* and /all/*          +-- log blocks: pose, status, kalman_preflight
      | topics   /cfX/cmd_*, /all/cmd_*     |   come back as /cfX/pose, /cfX/status,
      |                                     |   /cfX/kalman_preflight (+ connection_statistics)
 [crazyflie_py  Crazyswarm()] <-------------+
  used by every show script
      ^
      | subprocess `ros2 ...` ONLY, never in-process rclpy
 [console/ web GUI :8077] --> launches launch.py, shows, scans, e-stop
                              (/api/estop -> ros2 service call /all/emergency)
```

Processes started by `ros2 launch crazyflie launch.py` (read from `launch.py`):
`motion_capture_tracking_node` (when `backend != sim` and `mocap:=True`); one
`crazyflie_server` variant chosen by `backend` (`cpp` default, `cflib`, `sim`), further
gated by `server:=`; `rviz2` (`rviz:=True` default); `preflight_kalman_plotter.py`
(`preflight:=True` default); `foxglove_bridge` (`foxglove:=True` default); `teleop` +
`joy_node` (`teleop:=True` default); optional `gui.py` (`gui:=False` default).
**`server:=False`** leaves mocap + RViz + preflight GUI up with **no radio owner and
nothing armable** — that is exactly the state `scripts/sync_initial_positions.py` needs.
`use_sim_time` is set on rviz/preflight/foxglove/gui only when `backend == sim`.

### 2. Topic and service table

Direction is from the server's point of view. SENSOR = SensorDataQoS (BEST_EFFORT).
DEF = rclcpp SystemDefaultsQoS (RELIABLE, as read).

| Name | Type | Dir | Rate | QoS | Publisher / provider |
|---|---|---|---|---|---|
| `/poses` | `motion_capture_tracking_interfaces/NamedPoseArray` | in to server, preflight GUI, shows, sync script | mocap 50 Hz; the server warns outside `warning_if_rate_outside` [40,60] (cfg) | SENSOR, keep_last 1, deadline from `motion_capture.yaml` `topics.poses.qos.deadline` (cfg). **A RELIABLE subscriber is incompatible and receives NOTHING** | `motion_capture_tracking_node` (default) or `pose_bridge.py` (alt path) |
| `/tf` | `tf2_msgs/TFMessage` | out of mocap node, in to RViz/Foxglove | per frame | default | mocap node (TransformBroadcaster); in sim, `visualization/rviz.py` |
| `/cfX/pose` | `geometry_msgs/PoseStamped` | out | 10 Hz (cfg) | depth 10 | server, from the drone's **onboard estimate** via a log block. NOT mocap truth — never copy it into `initial_position` |
| `/cfX/status` | `crazyflie_interfaces/Status` | out | 1 Hz (cfg) | depth 10 | server (log block); carries `supervisor_info` and the rx counters. Shows subscribe with SENSOR QoS (`preflight.py`) |
| `/cfX/kalman_preflight` | `crazyflie_interfaces/LogDataGeneric` | out | 5 Hz (cfg) | depth 10 | server, custom log topic (22 of the 26 B block budget). Feeds the preflight GUI |
| `/cfX/connection_statistics`, `/all/connection_statistics` | `crazyflie_interfaces/ConnectionStatisticsArray` | out | `warnings.frequency` (1 Hz, cfg), only while `publish_stats: true` (it is) | depth 10 | server |
| `/cfX/robot_description` | `std_msgs/String` | out | latched once | QoS(1) transient_local | server and sim server |
| `/cfX/scan`, `/cfX/odom` | `sensor_msgs/LaserScan`, `nav_msgs/Odometry` | out | only if enabled in the yaml | depth 10 | server |
| `/cfX/cmd_full_state` | `crazyflie_interfaces/FullState` | in | show-defined; escort `rate_hz` default 20 Hz per drone | DEF; the `crazyflie_py` publisher is depth 1 | `crazyflie_py` `cmdFullState()`; server calls `sendFullStateSetpoint` |
| `/all/cmd_full_state` | `FullState` | in | as above | DEF | `crazyflie_py` allcfs |
| `/cfX/cmd_position`, `cmd_hover`, `cmd_velocity_world`, `cmd_vel_legacy` | `Position`, `Hover`, `VelocityWorld`, `geometry_msgs/Twist` | in | caller-defined | DEF | `crazyflie_py` / teleop. The **sim** server subscribes to only `cmd_vel_legacy`, `cmd_hover`, `cmd_full_state` |
| `/cfX/{takeoff,land,go_to,arm,emergency,start_trajectory,upload_trajectory,notify_setpoints_stop}` | `Takeoff, Land, GoTo, Arm, std_srvs/Empty, StartTrajectory, UploadTrajectory, NotifySetpointsStop` | service, in | on demand | default | server, **all on `callback_group_cf_srv`**. `UploadTrajectory` carries success/message; `StartTrajectory` has no response fields |
| `/all/{takeoff,land,go_to,arm,emergency,start_trajectory,notify_setpoints_stop}` | same types | service, in | on demand | default | server, on `callback_group_all_srv_`, via `broadcaster_`. **No `/all/upload_trajectory`.** `/all/emergency` registers LAST: its existence means the server finished connecting |
| `/crazyflie_server/set_parameters` (+get/list/describe) | `rcl_interfaces` | service, in | on demand | default | server. Firmware param pushes (`<cf>.params.*`, `all.params.*`) run **synchronously** in an `add_on_set_parameters_callback` |
| `/mocap/<body>/pose` | `geometry_msgs/PoseStamped` | alt path, in to `pose_bridge.py` | per Motive frame | `rclcpp::QoS(1000)` | `natnet_ros2` (namespace `mocap` by default) |

**How the server consumes `/poses`** (read in `posesChanged`): each message is
timestamped for the mocap-rate warning; each rigid body's z feeds that drone's
telemetry-watchdog airborne guard; bodies whose name matches a drone are split by NaN
orientation into position-only or full-pose lists and sent with `sendExternalPositions` /
`sendExternalPoses` on every broadcaster. **Mocap reaches the drones whether or not any
show is running** — the server is the fusion path, not the show.

### 3. Alternative mocap path: natnet_ros2 + pose_bridge.py

```
Motive --NatNet--> natnet_ros2 --/mocap/<body>/pose--> pose_bridge.py --/poses--> (same consumers)
```

Read from source: `natnet_ros2.launch.py` defaults `namespace:=mocap`; `pose_bridge.py`
subscribes to `/<ns>/<name>/pose` for every ENABLED drone in `crazyflies.yaml` and
republishes a `NamedPoseArray` on `/poses` with BEST_EFFORT / keep_last(1) / VOLATILE and
deadline `1/PUBLISH_HZ` (`PUBLISH_HZ = 50.0`). Poses older than `stale_after_s` (0.25 s)
are **dropped, not repeated** — the server force-fuses mocap, so a frozen pose looks like
a drone holding perfectly still.

Use this only when the vendored `motion_capture_tracking` cannot be used; it is not the
default. **Never run both publishers on `/poses`.** Trap (CLAUDE.md): with an *empty*
namespace, `natnet_ros2` and `crazyflie_server` both publish `/<name>/pose`, so the
bridge feeds the drones' own EKF estimates back as mocap truth. The bridge warns when the
namespace is empty.

### 4. What a show adds on top

A show is a `crazyflie_py` script run with `ros2 run`. It is **not** part of the launch.
`Crazyswarm()` enumerates drones by their `StartTrajectory` services and creates, per
drone, clients for `emergency, takeoff, land, go_to, upload_trajectory, start_trajectory,
notify_setpoints_stop, arm`, plus `set_parameters`/`get_parameters` on
`/crazyflie_server`, subscriptions to `/cfX/status` and `/cfX/pose` (depth 10), and
publishers for `cmd_full_state`, `cmd_position`, `cmd_hover` (depth 1); also the `/all/*`
clients and `all/cmd_full_state`. The constructor **blocks on `wait_for_service` for
`/all/emergency`**, so a show started with no server hangs right there.

Two control styles:

1. **Onboard high-level commander** (carousel, constellation, `multi_trajectory*`):
   upload piecewise polynomials once (`upload_trajectory` — radio-heavy; see the
   upload-wedge entry in CLAUDE.md), then `start_trajectory` / `go_to` / `takeoff` /
   `land`, mostly via `/all/*`. The drone flies itself; the host sends little in flight.
2. **Host-streamed setpoints** (escort): `cmd_full_state` per drone at the show's
   `rate_hz` (20 Hz default). Streaming leaves the high-level commander, so
   `notify_setpoints_stop` must be called before any later `go_to` / `land`.
   `cmdFullState` is used rather than `cmdPosition` because the sim server has no
   `cmd_position` subscription at all.

Shows also read `/poses` directly (escort's `PoseCache`, `sync_initial_positions.py`, the
preflight checks) and must therefore use `qos_profile_sensor_data`. **Every show must
call `crazyflie_shows/preflight.py`** (the supervisor-LOCKED go/no-go read from
`/cfX/status`) **and `abort.py`** (`take_signals()`, so Ctrl-C / SIGTERM lands the fleet
instead of dying in an already-shut-down rclpy context). `docs/WRITING-A-SHOW.md` is the
guide for adding your own.

### 5. The SIM path (`backend:=sim`)

```
launch.py (no mocap node, no /poses) --> crazyflie_sim crazyflie_server (Python, cffirmware SIL)
   same /cfX/* and most /all/* SERVICES; cmd_full_state / cmd_hover / cmd_vel_legacy topics
   outputs: /tf via the rviz visualization, /cfX/robot_description.  Simulated time.
```

Read from source: `launch.py` skips `motion_capture_tracking_node` when `backend == sim`;
`crazyflie_sim/crazyflie_server.py` has no `/poses` subscription and no `/cfX/pose` or
`/cfX/status` publisher (a grep of `crazyflie_sim` found only a TransformBroadcaster).
Consequences: readers of `/poses` (escort's `PoseCache`, `sync_initial_positions.py`) see
nothing, and that miss looks **identical** to a wrong-QoS subscriber (the escort source
says so in a comment); the preflight GUI's mocap/status checks have no data. The sim
clock runs ~4x slower than wall time (CLAUDE.md), so the trajectory demos need
`--ros-args -p use_sim_time:=true` — **hardware runs WITHOUT it** (and escort lights are
off under `use_sim_time`). `cffirmware` must be importable
(`scripts/setup_sim_firmware.sh`; conda shadows it). The sim has no radio and no UDP
1511, so the resource conflicts below do not apply; the mutually exclusive callback group
is a property of the C++ server only.

### 6. Scarce resources and who owns them

| Resource | Owner | What a second claimant does | How to detect |
|---|---|---|---|
| Crazyradio dongle (`radio://0/80/2M`) | `crazyflie_server` (cpp) | One owner at a time. The link library skips any radio whose serial it cannot query, so the server's stick is invisible to other processes and a second dongle is `devId 0` to them. `color_led_cflib.py` and `deck_check.py` need the server STOPPED | `./scripts/scan_fleet.sh` before every launch; `./scripts/which_server.sh` |
| UDP 1511 (NatNet data) | `motion_capture_tracking_node` | A leftover process means two `SO_REUSEPORT` sockets; the kernel hands each datagram to one, the mocap node starves and hangs silently in `connect()`. Ping to Motive proves nothing (unicast != multicast) | `ss -uanp \| grep :1511`; `pgrep -f motion_capture_tracking` |
| Motive transmission type | read ONCE at connect | Changed after launch = unseen. A frozen mocap node ignores SIGINT; launch escalates to SIGKILL and can leave the node behind | full relaunch; check `pgrep` for leftovers |
| `callback_group_cf_srv` (fleet-wide, mutually exclusive) | created once in `CrazyflieServer`, handed to every `CrazyflieROS` | Carries every drone's per-drone services, the telemetry-watchdog timer AND the 1 ms `spin_once` that is the only place log data is dispatched. One stuck wait on one drone freezes **all** drones' per-drone services and **all** telemetry. Worst-case tier-2 rebuild ~14 s fleet-wide (CLAUDE.md, OPEN item) | a log line `[cfX] upload_trajectory(...)` with nothing after it; every telemetry topic at 0 Hz |
| `callback_group_all_srv_` | the `/all/*` services and the warnings timer | Separate group, going through `broadcaster_`, never touching a stuck drone's `Crazyflie` object — which is why `/all/emergency`, `/all/land` and `/all/takeoff`, the console E-STOP and the preflight GUI's `e` key keep working through a wedge | verified per CLAUDE.md |
| Drone-side 1 s receive timeout | each drone's firmware (`radiolink.c` -> `log.c`) | One second without **receiving** anything makes the drone delete every log block and flush its CRTP queues, silently, while still acking at the radio level. Mitigations: `keepalive_frequency` (insurance only) and the server's telemetry watchdog | the failure table below |
| ROS domain | the lab | Domain 0 is shared; a colleague's `/all/takeoff` is visible here. Set `ROS_DOMAIN_ID` before flying | `ros2 node list` |
| Conda / venv | must be absent | ROS nodes run `/usr/bin/python3`; conda gives `No module named '_cffirmware'` and invalid message types. `build.sh` and `console/run.sh` strip it; a plain `ros2 launch` does not | `which python3` |

### 7. Failure directions: what dies, and what the operator actually sees

| Broken link | What goes dead | Symptom | Source |
|---|---|---|---|
| Motive not streaming / wrong transmission type / Motive PC address drifted | `/poses`; the drones get no external position | `/poses` has 0 publishers; launch aborts loudly if `auto` finds no Motive; preflight mocap-Hz flatlines; server warns outside [40,60] | CLAUDE.md; `launch.py` |
| UDP 1511 held by a stale process | mocap node starves, hangs in `connect()`, never appears in `ros2 node list` | no crash, no `/poses`, and ping to Motive still works | CLAUDE.md (confirmed cause) |
| Apt `motion_capture_tracking` shadowing the vendored one | `/poses` (hard-coded foreign IP) | silent; the launch looks completely normal | CLAUDE.md, VENDORED.md |
| A subscriber uses default RELIABLE QoS on `/poses` | that subscriber only | receives nothing, no error — this was a real bug here | CLAUDE.md; escort / sync comments |
| Rigid body created with the wrong yaw | nothing at rest (position force-fused at `extPosStdDev` 1e-3, ~1 mm) | fly-away in flight; the preflight GUI banner / `err.yaw` catches it (+-5 deg fly, 5-15 fix, >20 no fly) | CLAUDE.md |
| First enabled drone off, or wrong datarate in its URI | the whole server, during connect | blocks **silently**: no error, no `/all/*` services, a show hangs in `Crazyswarm()`, needs SIGKILL | CLAUDE.md |
| One drone's log blocks deleted by the drone (>= 1 s receive gap) | that drone's `/pose`, `/status`, `/kalman_preflight` — exactly 0.00 Hz | one drone silent in the GUI while link stats look perfect and its services still answer; the telemetry watchdog recovers it in ~1.0 s (*meas* 2026-10-05), and only when mocap positively shows it on the floor | CLAUDE.md |
| Trajectory upload saturates the radio and a memory-write reply is flushed | `callback_group_cf_srv`: all per-drone services and all telemetry | all drones stop mid-upload; preflight shows `takeoff (cfX): timed out waiting for response`; `/all/*` still works. Fixed with bounded waits; `bad_trajectories_` then makes `start_trajectory` refuse an unclean id | CLAUDE.md |
| A wedged server gets SIGINT | nothing | it does not exit; needs SIGKILL, and `ros2 launch` leaves the node behind | CLAUDE.md |
| Supervisor LOCKED after an e-stop | arming, fleet-wide or per drone | nothing flies; `supervisor_info` bit `0x40` set and `CAN_BE_ARMED` clear; **only a battery pull clears it**, and `arm()` is fire-and-forget so a refusal is otherwise invisible | CLAUDE.md |
| Silent `/cfX/status` | not a refusal | preflight counts it a NOTE, because the telemetry stall looks identical — watch that drone on takeoff | `crazyflie_shows/preflight.py` |
| Ctrl-C in a show | the rclpy context, before your handler runs | `allcfs.land()` dies with `failed to initialize wait set: the given context is not valid`; fixed by `take_signals()` in `abort.py` | CLAUDE.md |
| Server dead, or you are on another ROS domain | everything downstream | `ros2 service call` has no timeout and prints nothing; the console reports e-stop NOT CONFIRMED within 3 s and abandons pending calls at 15 s | CLAUDE.md, console |
| Console backend not restarted after a code change | the new routes | `{"error": "not found"}`; the page shows a restart banner and the e-stop falls back to `/api/run` | CLAUDE.md, console/README.md |
| Sim: there is no `/poses` | every reader of `/poses` | escort / sync wait forever, looking exactly like a QoS bug | `escort_show.py` |
| Same-process DDS loopback (this rig) | a node never hears its **own** `/parameter_events`; a fresh rclpy process may never finish discovery | upstream `ros2 param set` silently did nothing; `Crazyswarm()` can hang on `all/emergency`. Hence the callback-based param pushes and the CLI-driven `led.sh` | CLAUDE.md |

**Blast radius.** Mocap loss degrades position hold but leaves the services alive. Radio
loss or a wedged server takes out everything except the `/all/*` e-stop path — which only
helps while the server process is still running. A single drone's log stall is local and
self-heals. A wedged `cf_srv` group is fleet-wide.

### 8. Where the console sits

The console (`console/run.sh`, default `127.0.0.1:8077`, routes under `/api/`) is **not in
the data path**. `procs.py` runs the same `ros2 ...` commands a human would, each child in
its own session; `health.py` turns documented silent failures into graph nodes (UDP 1511
starvation, apt mocap shadowing, server blocked mid-connect, datarate mismatch, conda
python). `/api/estop` runs `ros2 service call /all/emergency` and reports confirmed or NOT
CONFIRMED within 3 s. Removing `console/` changes nothing else. User guide:
`docs/CONSOLE.md`; internals: `console/README.md`.

### 9. Caveats to read with the diagram

- Do not copy any rate above into another document. The YAML is the source, and rates
  have drifted here before.
- The diagram shows **one** dongle. A second dongle is historical and brings its own
  channel rules (channels >= 2 apart at 2M).
- Only the **cpp** `crazyflie_server` has the telemetry watchdog, the airborne gate and
  the bounded upload. The sim and cflib backends do not.

### What this map does NOT claim to have verified

- The `/tf` publish rate out of `motion_capture_tracking` — assumed to follow the mocap
  frame rate, not measured.
- Whether the sim server exposes `/all/arm` or `/all/notify_setpoints_stop` (a grep found
  only `/all/` takeoff, land, go_to, start_trajectory, emergency).
- QoS compatibility of the `/cfX/cmd_*` subscriptions with the `crazyflie_py` publishers
  was read in source, not tested on the wire.
- `/cfX/pose` 10 Hz, `/cfX/status` 1 Hz and `/cfX/kalman_preflight` 5 Hz are **config
  values** from `crazyflies.yaml` on 2026-10-08, not live measurements.
- The 20 Hz escort stream rate is the `escort_show` docstring default, not a hardware
  capture. That the carousel and constellation shows use only onboard trajectories is
  inferred from CLAUDE.md and the planner docs, not from a line-by-line read.
- `poses_version` (NamedPoseArray v1 vs V2) was not checked in `motion_capture.yaml`.
- The teleop / `joy_node` topic flow was not traced; teleop is listed as launched, not
  mapped.
- The cflib backend (`crazyflie_server_py`) was not examined for whether it shares these
  names.

---

## Where do I look when X is broken

| Symptom | Part of the graph | Doc that covers it |
|---|---|---|
| `/poses` has no publishers, or no data | mocap node ↔ Motive (§1, §6 UDP 1511) | `docs/MOCAP.md`, `docs/TROUBLESHOOTING.md` |
| Mocap node missing from `ros2 node list`, no crash | UDP 1511 starvation (§6) | `docs/TROUBLESHOOTING.md`, `src/motion_capture_tracking/VENDORED.md` |
| Drone drifts or flies away although position looks perfect | rigid-body yaw, `/poses` → server fusion (§7) | `docs/MOCAP.md` |
| Launch hangs with no error and no `/all/*` services | server connect loop, first unreachable drone (§7) | `docs/RUNNING.md` (run `./scripts/scan_fleet.sh` first), `docs/TROUBLESHOOTING.md` |
| One drone silent: `/cfX/pose`, `/status`, `/kalman_preflight` all 0.00 Hz | drone-side log-block deletion + telemetry watchdog (§6, §7) | `CLAUDE.md` (the full measurement history), `docs/TROUBLESHOOTING.md` |
| All drones stop publishing during a trajectory upload | `callback_group_cf_srv` wedge (§6) | `CLAUDE.md`; recover per `docs/TROUBLESHOOTING.md` |
| Nothing arms, nothing flies, no error | supervisor LOCKED (§7) | `runbooks/*.runcard.md` (preflight step), `docs/RUNNING.md` §C |
| My script subscribes to `/poses` and gets nothing | QoS incompatibility (§2) | `docs/WRITING-A-SHOW.md`, `docs/TROUBLESHOOTING.md` |
| Ctrl-C left the drones flying | show signal handling, `abort.py` (§4, §7) | `docs/WRITING-A-SHOW.md` |
| Sim: the show hangs waiting for poses | sim has no `/poses` (§5) | `docs/RUNNING.md` (sim section), `docs/WRITING-A-SHOW.md` |
| Sim: server dies at the first `startTrajectory` | `crazyflie_sil.py` arity probe (§5) | `CLAUDE.md` gotcha; `docs/RUNNING.md` |
| Console button answers `{"error": "not found"}` | console backend not restarted (§8) | `docs/CONSOLE.md`, `console/README.md` |
| Which server is even running? hardware, sim, both? | §1 vs §5 | `./scripts/which_server.sh`, `docs/RUNNING.md` |
| Build produces an empty `install/`, or messages are "invalid" | `set -u` + ROS setup.bash, or conda shadowing (§6) | `README.md` setup section, `CLAUDE.md` gotchas |
| A show refuses to fly: envelope / separation | planners against `safety.ARENA_*` (§4) | `src/crazyswarm2/crazyflie_shows/SHOW_GUIDE.md`, `CONSTELLATION.md`, `ESCORT.md` |

---

## Single sources of truth

Four rules hold this repo together. Every one of them exists because a copy drifted and
cost a flight.

1. **One `crazyflies.yaml`**, at `src/crazyswarm2/crazyflie/config/`. It is what the
   server reads, what every planner reads, and what `sync_initial_positions.py` writes. A
   second copy lived under `crazyflie_shows` until 2026-10-02 and drifted — the planner
   verified one fleet while the server flew another. Do not reintroduce one; use
   `crazyflies_yaml_file:=` only for a genuine one-off fleet, and expect the
   **FLEET YAML OVERRIDE** banner when you do.
2. **The arena constants live in `crazyflie_shows/safety.py`**, measured, not guessed:
   `ARENA_CENTRE`, `ARENA_RADIUS_TESTED`, `ARENA_RADIUS_LOST`, and the derived
   `ARENA_RADIUS_PLAN`. Every show checks its plan against those, from that file. Do not
   carry a private room centre inside a demo — `multi_trajectory_formation.py` did
   exactly that and planned 0.52 m outside the envelope.
3. **Live numbers are PRINTED, never transcribed.** The fleet roster, the tuned
   constants, the topic rates: run the command.
   `./scripts/scan_fleet.sh --list` for the fleet, `plan_show` / `plan_constellation` /
   `plan_escort` after every position sync, `ros2 topic hz` for rates. Prose copies of the
   roster have drifted six times in this repo.
4. **Operator runcards live in `runbooks/`; show DESIGN docs stay inside
   `src/crazyswarm2/crazyflie_shows/`.** That split is deliberate: the show package is
   meant to lift out whole and still carry its own design rationale, while the flight-day
   cards belong with the rig.

And two structural ones: `src/` **is** committed (editing it is how you change the rig),
while `build/`, `install/`, `log/` and `core.*` are not — and `console/` is removable, so
nothing in `src/`, `launch.py`, `CMakeLists.txt` or `setup.sh` may ever depend on it.
