# RUNNING — launch flows

Activate the workspace in every terminal first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/CrazySwarm2/install/setup.bash
```

> Config files referred to below as `config/<name>.yaml` live at
> `src/crazyswarm2/crazyflie/config/<name>.yaml`. Edit them there and rebuild.

---

## A. Simulation (no hardware, no mocap)

The fastest way to verify the install. Uses the built-in `np` backend.

> **One-time prerequisite:** the sim imports the `cffirmware` bindings. If you see
> `ModuleNotFoundError: No module named 'cffirmware'`, run
> `./scripts/setup_sim_firmware.sh` once (see README → Setup Step 4).

```bash
ros2 launch crazyflie launch.py backend:=sim
# in another terminal, run an example against the sim:
ros2 launch crazyflie_examples launch.py script:=hello_world
```

> Running the multi-drone trajectory demos against the sim **requires**
> `--ros-args -p use_sim_time:=true` — the sim clock runs ~4x slower than wall
> time and the script otherwise races ahead of the physics. See
> [Section B → Multi-drone trajectory demos](#multi-drone-trajectory-demos).

RViz opens by default (`rviz:=false` to disable) and shows the simulated drone.
The preflight GUI (section C) also opens by default; in sim there is no mocap so
its mocap panels stay empty — pass `preflight:=False` if you don't want it.
Backend/controller/visualization options are in `config/server.yaml`
(`sim:` section).

## B. Hardware flight with OptiTrack

This is the real-world takeoff → hover → land flow for one drone.

The `motion_capture_tracking` node started by `launch.py` connects **directly** to
Motive over the network and publishes `/poses` — you do **not** start a separate
driver or bridge for this path.

### Prerequisites

1. Calibrate the cameras and define a rigid body per drone — see
   [docs/MOCAP.md](MOCAP.md). The rigid-body name must match the robot key in
   `config/crazyflies.yaml` (e.g. `cf1`).
2. In **Motive → Settings → Streaming**, set the data signal type to
   **Multicast** and the rate to **50 Hz**. This **must match**
   `config/motion_capture.yaml` in the crazyswarm2 package
   (`type: "optitrack"` — the open parser, **required on Wi-Fi** and fine on
   LAN — and `poses.qos.deadline: 50.0`). A `type: "optitrack_closed_source"`
   option exists but hangs **permanently** after a Wi-Fi multicast stall — don't
   use it. A mismatch here is the most common reason the drone won't localize.

### Fly

```bash
# terminal 1 — start the Crazyflie server (also starts mocap tracking, RViz,
# the preflight GUI and the Foxglove bridge — all on by default)
ros2 launch crazyflie launch.py

# run the preflight checklist in the GUI — see section C below

# terminal 2 — takeoff, hover, land
ros2 run crazyflie_examples hello_world
```

The drone arms, takes off, hovers, then lands and disarms. Don't skip the
preflight check: it is the only thing that catches a rotated Motive rigid body
before it becomes a fly-away (section C, "The orientation warning").

### Adjusting the flight (hello_world.py)

The behaviour is set by a few values at the top of and inside
`crazyflie_examples/hello_world.py`:

| What | Where in the script | Default |
|------|--------------------|---------|
| **Takeoff duration** (time to climb) | `TAKEOFF_DURATION` | `5.0` s |
| **Hover duration** (time held in the air) | `HOVER_DURATION` | `5.0` s |
| **Hover height** (target altitude) | `cf.takeoff(targetHeight=0.5, ...)` | `0.5` m |
| **Landing height** (touchdown target) | `cf.land(targetHeight=0.03, ...)` | `0.03` m |
| **Landing duration** (time to descend) | `cf.land(..., duration=3.0)` | `3.0` s |

Reading the script: it builds a `Crazyswarm()`, grabs the first drone, then
`arm(True)` → `takeoff(targetHeight, duration)` → `sleep(TAKEOFF + HOVER)` →
`land(targetHeight, duration)` → `arm(False)`. So raise `targetHeight` in
`takeoff()` to hover higher, increase `HOVER_DURATION` to hover longer, and tune
the `land()` arguments for a slower/softer or lower touchdown. Start low (≤0.5 m)
for first flights.

### Visualization

Two viewers, both optional:

- **RViz** — on by default (`rviz:=false` to disable). It shows two frames
  per drone: `cf1` (the **onboard EKF** state estimate) and `cf1_mocap` (the raw
  **mocap** pose). They should sit almost on top of each other; a large or growing
  gap means the estimator and mocap disagree (bad calibration, marker/rigid-body
  issue, or estimator not converged). Rotated axes between the two frames mean
  the rigid body was defined with a yaw offset — the preflight GUI (section C)
  quantifies this as `err.yaw`.
- **Foxglove** — on by default (`foxglove:=True`), but needs the bridge package
  installed (`sudo apt install ros-$ROS_DISTRO-foxglove-bridge`). It's not a
  window: open the Foxglove Studio app and connect to `ws://localhost:8765` to
  inspect topics, poses, and TF live.

### Verifying the pose pipeline

```bash
ros2 topic hz /poses          # mocap tracking publishing? should be ~50 Hz
ros2 topic echo /cf1/pose --once   # onboard pose estimate (firmware logging)
ros2 topic echo /cf1/status --once # battery / supervisor / link health
```

If `/poses` is silent → fix Motive streaming (Multicast + 50 Hz, Broadcast Frame
on) or the hostname/IP in `config/motion_capture.yaml` — it must be the Motive
PC's **current** address, which changes when you switch between Wi-Fi and LAN
([MOCAP Section 5](MOCAP.md#5-networking-mocap-over-a-router-lab-setup)). If `/poses` **was**
flowing and stopped, check `ros2 topic info /poses` — 0 publishers means the
mocap node is dead or starved (a leftover process on UDP 1511 is the confirmed
cause on this rig). See [TROUBLESHOOTING](TROUBLESHOOTING.md#mocap-pipeline).

> **Alternative (open NatNet driver).** If you stream via the open-source
> `natnet_ros2` driver instead of the closed-source direct client, start it and
> the bridge separately: `ros2 launch natnet_ros2 natnet_ros2.launch.py` then
> `python3 ~/CrazySwarm2/pose_bridge.py` (republishes per-body poses to `/poses`
> at 50 Hz). The flight steps above are otherwise identical.

### Multi-drone trajectory demos

Two whole-fleet scripts (`crazyflie_examples`), both flown on this rig. Every
enabled drone flies, so clear the preflight checklist for **all** of them and
make sure each drone sits at **its own** `initial_position`
([MOCAP Section 2b](MOCAP.md#2b-setting-initial_position-from-poses)) — wrong-corner
placement means crossing `goTo` paths and caused a real collision 2026-08-04.

```bash
# in a second sourced terminal, with the server (terminal 1) running and the
# preflight checklist cleared for every drone.
# hardware — NO extra args
ros2 run crazyflie_examples multi_trajectory
ros2 run crazyflie_examples multi_trajectory_formation

# simulation — the use_sim_time flag is REQUIRED (see the callout below)
ros2 run crazyflie_examples multi_trajectory --ros-args -p use_sim_time:=true
ros2 run crazyflie_examples multi_trajectory_formation --ros-args -p use_sim_time:=true
```

- **`multi_trajectory`** — arms, takes off to 1 m, every drone flies **only the
  short `traj1.csv` (~25 s)** in formation (traj0 was dropped for a **~50 s
  total flight**), then a return-to-home `goTo` (+0.75 m hover over each
  drone's own `initial_position`) and a **slow 4 s landing**
  (`targetHeight=0.04`). Because all drones fly the *same relative*
  trajectory, the flight preserves the start separation — hence the ≥ 1 m
  spacing rule.
- **`multi_trajectory_formation`** — same start (arm, takeoff, move to the
  start hover positions — it **no longer flies `traj1.csv`**; plain
  `multi_trajectory` still does, unchanged), then a formation dance in six
  phases:
  1. **Formation waypoint tour** (3 waypoints + return, rigid group moves):
     the same xy offset (`WAYPOINT_OFFSETS` = (0.6, 0), (−0.6, 0.5),
     (0, −0.6), then back to (0, 0)) is applied to **every** drone's own
     start hover position, so the group translates rigidly and the
     separation stays exactly the start spacing (1.36 m with the current
     yaml) on all four legs. Legs are distance-scaled
     (`max(2.0, leg / 0.5)` s → 2.0/2.6/2.5/2.0 s); max excursion ~2.14 m
     from `ROOM_CENTER`, inside the ~2.24 m orbit clearance.
  2. **Gather** onto a regular **n-gon** (a pentagon with the 5-drone fleet)
     of radius 0.8 m around the swarm center. The n-gon's phase offset is
     auto-optimized before takeoff (1° sweep plus min-distance slot
     assignment, maximizing the worst-case pairwise separation along the
     simultaneous straight-line gather paths); adjacent slots sit 0.94 m
     apart at n=5.
  3. **Rotate** the n-gon a full **+360° in ONE continuous smooth motion**:
     each drone flies an uploaded circle trajectory (id 1, **10 s**,
     ~0.50 m/s tangential) that starts and ends at its own slot — **not**
     stepped `goTo`s.
  4. **Morph** onto a **triangle + tail-pair** formation (5 slots: apex, two
     rear corners, two tail drones; min-distance assignment, no crossings).
  5. **Orbit**: the whole formation translates rigidly onto a **1.2 m-radius
     circle around the room center** `ROOM_CENTER` **(0.0467, −0.1037)** and
     flies one full revolution (shared uploaded circle, id 2, **12 s**,
     ~0.63 m/s tangential, ~0.33 m/s² centripetal — well inside the 1.3 m/s
     envelope) — every drone flies the *same* circle with `relative=True`,
     which makes it a rigid translation of the swarm. The shift onto the
     ring is a long move, so its `goTo` duration is **distance-scaled**:
     `max(2.0, longest_xy / 0.5)` s, i.e. ≤ 0.5 m/s average (~0.9 m/s
     rest-to-rest peak); ~2.36 s with the current yaml starts.
  6. **Return home via the pentagon**: a DIRECT return from the ring
     crossed paths at R=2.0 (1 route crossing; at R=1.2 it happens to be
     crossing-free, but the safe two-leg return is kept), so each drone
     first **expands back onto its own gather-pentagon slot**
     (distance-scaled `goTo`, ~2.50 s; verified 0 crossings, min
     separation 0.84 m), then flies **home from the pentagon** — the
     exact reverse of the gather (~2.0 s, min separation 0.84 m) — to its
     own `initial_position` + 0.75 m, then a slow **4.5 s** land
     (~0.16 m/s descent from the 0.75 m hover).

  Verified numbers: **min separation 0.84 m** (tail-to-tail in the
  triangle+tail formation); **CLEARANCE: the orbit sweeps up to ~2.24 m
  from `ROOM_CENTER`** (1.2 m orbit radius + ~1.04 m formation extent;
  shrank from ~3.05 m at the old 2.0 m ring) — keep the full
  ~2.24 m-radius circle clear of people and obstacles; **≈ 58.0 s
  takeoff → landed per trial**;
  formation altitude a constant **1.0 m** (`FORM_HEIGHT`). Per drone the demo
  uploads **12 trajectory pieces (6 rotation circle + 6 orbit circle;
  traj1's 16 dropped with the opening pattern)** ≈ 1.6 KB of the firmware's
  ~4 KB trajectory memory. Constants at the top of the script.

  Demo footage: see [`video/formation_demo_1.gif`](../video/formation_demo_1.gif)
  and [`video/formation_demo_2.gif`](../video/formation_demo_2.gif) (second
  clip trimmed to the first 60 s).

> **Sim needs `use_sim_time`.** The sim clock runs ~4x slower than wall time
> (no realtime pacing); without `--ros-args -p use_sim_time:=true` the script's
> sleeps run on wall clock and it **races ahead of the physics** (commands fire
> before the previous motion finished). On hardware run **without** the flag.

## C. Preflight GUI (preflight_kalman_plotter.py)

`launch.py` auto-starts `crazyflie/scripts/preflight_kalman_plotter.py`
(disable with `preflight:=False`). One window, one drone at a time — this is
the go/no-go check before every flight.

### What you see

- **Header** — big drone name + battery voltage, color-coded to the
  `crazyflies.yaml` thresholds (green > 3.8 V, orange 3.7–3.8 V, red < 3.7 V),
  plus `(charging)` / `ARMED` flags. Cycle drones with the ◀ Prev / Next ▶
  buttons or the left/right arrow keys (wrap-around; a `drone 2/3` indicator
  shows where you are). Only drones with `enabled: true` in `crazyflies.yaml`
  appear (`--config <path>` points the script at a different yaml; if the yaml
  can't be read it accepts everything and prints a warning).
- **2×2 plots**:
  1. **kalman telemetry** — 8 firmware vars from the `kalman_preflight` custom
     log topic (`kalman.stateX/Y/Z`, `motion.deltaX/Y`, `range.zrange`,
     `stateEstimateZ.vx/vy`), in raw firmware units.
  2. **pose** — stateEstimate x/y/z [m] and roll/pitch/yaw [deg] from
     `/<cf>/pose`.
  3. **connectivity** — mocap arrival rate [Hz] of this drone's entry in
     `/poses` (decays to 0 within ~1 s when mocap dies — that is the "mocap
     died" tell), plus radio RSSI and latency from `/<cf>/status`.
  4. **mocap − stateEstimate error** — position err.x/y/z/norm [m] as solid
     lines (left axis), orientation err.roll/pitch/yaw [deg] as dashed lines
     (right axis, wrapped to ±180°).

### The orientation warning (read this)

A red banner like `⚠ MOCAP ORIENTATION MISALIGNED: cf2 (yaw 150°)` appears when
**any** enabled drone — not just the one displayed — has a fresh orientation
error over 10° (`ORIENT_WARN_DEG` in the script; samples older than
`ERR_STALE_S` = 3 s are ignored). The error-panel title also turns red when the
displayed drone is the offender.

Why this matters: `locSrv.extPosStdDev = 1e-3` force-fuses the mocap
**position** into the onboard Kalman, so position error is always ~1 mm even
when the Motive rigid body was defined rotated. A yaw offset is invisible in
position at rest — and causes a fly-away in flight. `err.yaw ≈ 0` means the
rigid body is aligned with the drone's forward axis (see
[MOCAP Section 2](MOCAP.md#2-defining-rigid-bodies) for the rule when creating it).

Go/no-go: within **±5°** fly; **5–15°** fix the rigid body first; **> 20°** do
not fly.

### Buttons and keys

Two rows of buttons. The **upper row** acts on the selected drone only, via its
per-drone services:

> Record CSV | Takeoff (cfX) | Land (cfX) | Arm (cfX) | Disarm (cfX) | E-STOP (cfX)

The **lower row** broadcasts to all drones:

> Reset Kalman (all) — sets `all.params.kalman.resetEstimation` 1→0 |
> Takeoff (all) | Land (all) | Arm (all) | Disarm (all) | E-STOP (all)

Keys: `r` = reset kalman, `e` = **broadcast** e-stop (deliberately broadcast —
the panic key stops everything). Takeoff/land have **no** key bindings on
purpose. The takeoff/land setpoints are constants at the top of the script
(0.5 m over 3 s up, 0.03 m over 3 s down).

### Record CSV

**Record CSV** snapshots **every** enabled drone at 10 Hz to
`~/crazyswarm_ws/preflight_logs/preflight_<YYYYmmdd-HHMMSS>.csv` — 33 columns:
t, drone, mocap x/y/z, estimate x/y/z/rpy, err x/y/z/norm/roll/pitch/yaw, the 8
`kal_*` vars, rssi, latency_ms, mocap_hz, battery_v, pm_state,
supervisor_info. Missing values stay blank. The button shows a live row count
while recording, and the file is flushed every tick.

### Preflight checklist

1. `ros2 launch crazyflie launch.py` — wait for the drones to appear in the GUI.
2. Per drone (cycle with the arrow keys):
   - no red misalignment banner;
   - mocap rate steady (~50 Hz) in the connectivity panel;
   - RSSI / latency stable;
   - error dashed lines near 0°, err.norm at mm level;
   - battery green.
3. **Reset Kalman (all)**.
4. Arm, then a short test hop with the per-drone Takeoff / Land.
5. Fly your mission script.

## D. Manual / teleop flight

Two paths. **`teleop_xbox` is the one flown on this rig** — a geofenced
position teleop with mocap safety interlocks; the upstream teleop node is kept
as the alternative.

### teleop_xbox (geofenced position teleop)

```bash
# terminal 1 — server WITHOUT the built-in teleop: only one process may own
# the controller and the drone. teleop:=False also stops joy_node — the
# script reads /dev/input/js0 directly, no joy_node needed.
ros2 launch crazyflie launch.py teleop:=False

# terminal 2 — fly the first enabled drone
ros2 run crazyflie_examples teleop_xbox
```

Controls (Xbox layout): **A** = take off (to 0.5 m), **Back/View** = land,
**B** = **EMERGENCY** — cuts motors instantly, the drone drops and needs a
physical reset. Left stick moves horizontally, right stick Y is up/down,
right stick X yaws, Ctrl-C lands then exits. The sticks steer a **position
setpoint**, not velocity — release them and the drone holds position.

Safety envelope (constants at the top of
`crazyflie_examples/teleop_xbox.py`):

| What | Where in the script | Default |
|------|--------------------|---------|
| **Geofence** (auto-land past this radius from (0,0)) | `FENCE_RADIUS` | `2.0` m |
| **Target clamp** (sticks can't push the setpoint past this) | `TARGET_RADIUS` | `1.8` m |
| **Height limits** (setpoint clamped) | `Z_MIN` / `Z_MAX` | `0.15` / `1.5` m |
| **Mocap staleness → auto-land** | `POSE_TIMEOUT` | `0.5` s |
| **Max setpoint lead over the measured position** (anti-windup) | `MAX_LEAD` | `0.8` m |

It subscribes to `/poses` with **sensor QoS** (the topic is best-effort; a
default reliable subscription silently receives *nothing*), refuses to take
off without a fresh mocap pose for its drone, and treats a stale pose in
flight as a fly-away risk (auto-land).

> Verify the controller mapping **without flying anything**:
> `ros2 run crazyflie_examples teleop_xbox --joytest` prints live axis/button
> values straight off `/dev/input/js0`.

### Built-in teleop (upstream)

```bash
ros2 launch crazyflie launch.py backend:=cpp teleop:=True
```

Gamepad mapping (buttons, axes, limits) is in `config/teleop.yaml`. Defaults:
takeoff = start, land = back, emergency = red, arm = yellow.

## E. Shows (crazyflie_shows)

Three verified multi-drone shows. Each has a **runcard** — the commands in
order with every argument — and a design document explaining why the numbers
are what they are.

| show | runcard | why | run |
|---|---|---|---|
| carousel, ~63 s | [CAROUSEL](../src/crazyswarm2/crazyflie_shows/runcards/CAROUSEL.runcard.md) | `SHOW_GUIDE.md` | `swarm_show` |
| constellation, 76 s | [CONSTELLATION](../src/crazyswarm2/crazyflie_shows/runcards/CONSTELLATION.runcard.md) | `CONSTELLATION.md` | `constellation_show` |
| escort, reactive | [ESCORT](../src/crazyswarm2/crazyflie_shows/runcards/ESCORT.runcard.md) | `ESCORT.md` | `escort_show` |

Start at
[`runcards/README.md`](../src/crazyswarm2/crazyflie_shows/runcards/README.md),
which carries the rules common to all three: deriving the address scan from the
yaml, and what an E-STOP costs.

Every show follows `plan -> sim -> dry_run -> fly`, and each has a planner that
needs **no radio, no mocap and no drones** (`plan_show`, `plan_constellation`,
`plan_escort`). **Re-run the planner after every `sync_initial_positions.py`** —
the carousel sits at 99 % of its separation budget.

One difference worth knowing before you fly any of them: `constellation_show`
and `escort_show` handle Ctrl-C and land; **`swarm_show` does not** — stopping
the carousel in flight means the E-STOP.


## F. Useful checks

```bash
ros2 node list
ros2 topic list
ros2 service list | grep -E 'takeoff|land|go_to|arm'

# battery / link health (status topic must be enabled in crazyflies.yaml)
ros2 topic echo /cf1/status --once
ros2 topic echo /cf1/connection_statistics --once
```

> **If the `/all/*` services (takeoff/land/arm/emergency) never appear**, the
> server is **blocked forever, silently, on the first enabled drone that
> doesn't answer radio** (it connects drones in lexicographic `std::map`
> order). Any cause counts: a dead drone (cf6 went silent 2026-08-04), a wrong
> address, or a **datarate mismatch** — learned the hard way with cf11 (a `2M`
> URI on a drone whose radio runs at `1M`). No error is printed, and the hung
> server needs SIGKILL. **Go/no-go rule: scan every enabled address before
> every launch** (`ros2 run crazyflie scan --address 0xE7E7E7E7XX` — the
> current enabled fleet is `0xE7E7E7E701`, `0xE7E7E7E702`, `0xE7E7E7E703`,
> `0xE7E7E7E710`, `0xE7E7E7E714`) and fix
> the URI or disable the drone in `config/crazyflies.yaml`. See
> [TROUBLESHOOTING](TROUBLESHOOTING.md#crazyradio--drones) and — when running
> two dongles — the rules in
> [MOCAP Section 3c](MOCAP.md#3c-trim-drone-log-topics-to-protect-radio-bandwidth).

### Prop-spin ground test (`arming` example)

```bash
ros2 run crazyflie_examples arming   # server must be running
```

Arms every enabled drone and spins all four propellers at **low PWM**
(`SPIN_PWM = 10000`/65535, ~15% — hover needs roughly 38000+) for
`SPIN_SECONDS = 10.0` s, then stops and disarms. A brushed CF2.1 does **not**
idle-spin its props on arming (that's Bolt/brushless behavior), so this
drives the firmware's motor-test params (`motorPowerSet.*`, the same path as
cfclient's propeller test) for a visible motors-alive check. Motors are
always stopped in a `finally:` block, even on Ctrl-C.

> **Ground test only.** Props spin — keep the drone on the floor and fingers
> clear. The PWM is far below hover thrust, so the drone stays put.

### Color LED deck — status convention and manual control

Drones carrying the bottom Color LED deck (`colorLedBot`) double as a status
indicator. **Green LED = drone connected and ready for command. Red LED =
drones are in flight / a script is controlling them.** In detail:

- **Green** — connected and ready for command (the cpp server sets green on
  connect; the drone is idle, no script owns it).
- **Red** — in flight / under script control (`Crazyswarm()` turns the deck
  red for its lifetime; an `atexit` hook in `crazyswarm_py.py` restores green
  on exit — normal return, exception, or Ctrl-C).
- **Dark** — clean server shutdown (the server switches the LED off on
  disconnect; after a hard link loss it keeps its last color instead).

Manual control, two equivalent front-ends (number keys:
`0`=off `1`=green `2`=red `3`=yellow `4`=blue `5`=purple `9`=white):

```bash
./scripts/led.sh 3                        # one-shot: yellow, then exit
./scripts/led.sh                          # interactive: press number keys, q quits
ros2 run crazyflie_examples color_led 3   # same key map / color names
```

Both set the firmware param `colorLedBot.wrgb8888` (uint32 packed
`0xWWRRGGBB`) on every connected drone. They deliberately avoid
`Crazyswarm()`: `led.sh` goes through the **`ros2` CLI daemon** (`ros2 param
set`), which keeps the ROS graph warm, while `color_led` calls the server's
`/crazyflie_server/set_parameters` service directly — on this rig fresh rclpy
processes can stall in DDS discovery and `Crazyswarm()` hangs waiting on
`all/emergency` (see [TROUBLESHOOTING](TROUBLESHOOTING.md#ros-2--dds--parameters)).

### Runtime firmware parameters

`ros2 param set` now **reliably reaches the drones** — the vendored
`server.cpp` applies `<cf>.params.*` / `all.params.*` changes in an
on-set-parameters callback that runs synchronously inside the service call
(upstream's `/parameter_events` handler never saw the node's own events on
this rig, so runtime changes silently never left the PC). With
`firmware_params.query_all_values_on_connect: True` in `config/server.yaml`,
every firmware param is exposed at connect:

```bash
ros2 param set /crazyflie_server cf1.params.ring.effect 7   # per drone
ros2 param set /crazyflie_server all.params.kalman.resetEstimation 1   # broadcast
```

### Viewing a drone's topics (ROS_DOMAIN_ID)

In this setup **all drones share the same ROS domain** — they are **not** separated
by `ROS_DOMAIN_ID`. Each drone is separated by its **namespace** instead, e.g.
`/cf1/pose`, `/cf2/pose`. To narrow the topic list to one drone, filter by namespace:

```bash
ros2 topic list | grep cf1        # only cf1's topics
ros2 topic echo /cf1/pose         # echo a specific drone topic
```

`ROS_DOMAIN_ID` only matters in two cases:

1. **Your terminal sees no topics at all.** The shell's `ROS_DOMAIN_ID` must
   **match** the one the crazyflie stack was launched with (default `0`). In every
   terminal that needs to see the topics, set the same value (and a matching
   `ROS_LOCALHOST_ONLY`), then re-list:
   ```bash
   export ROS_DOMAIN_ID=<same value as the running stack>
   ros2 topic list
   ```
2. **Shared lab network / multiple users on one machine.** Pick a unique
   `ROS_DOMAIN_ID` (0–101) and `export ROS_LOCALHOST_ONLY=1` so you don't see — or
   accidentally command — someone else's robots. (See upstream crazyswarm2
   `docs2/usage.rst`.)

> The `natnet_ros2` helper GUI (`src/natnet_ros2/scripts/helper_node_r2.py`) has a
> domain-id selector — it sets which domain the **NatNet driver node** publishes on,
> i.e. the driver's domain, not a per-drone separation.

## G. Enabling extra telemetry logging (custom topics)

The drones can stream onboard firmware variables back as ROS topics. This is
configured under `all.firmware_logging.custom_topics` in
`config/crazyflies.yaml`. Several example blocks are shipped **commented out** —
enable them by uncommenting.

```yaml
all:
  firmware_logging:
    enabled: true
    custom_topics:
      kalman_preflight:   # enabled by default -> /cf1/kalman_preflight
        frequency: 5      # feeds the preflight GUI (section C) — keep it
        vars: [kalman.stateX, kalman.stateY, kalman.stateZ, motion.deltaX,
               motion.deltaY, range.zrange, stateEstimateZ.vx, stateEstimateZ.vy]
      # attitude:         # uncomment to enable -> /cf1/attitude
      #   frequency: 10
      #   vars: [stabilizer.roll, stabilizer.pitch, stabilizer.yaw]
      # battery:          # uncomment to enable -> /cf1/battery
      #   frequency: 1
      #   vars: [pm.vbat, pm.state]
```

The `kalman_preflight` block is what the preflight GUI plots — leave it enabled
(it uses 22 B of the 26 B log-block budget; every var must exist in the
firmware's log TOC or the cpp server aborts at connect).

**To enable a log topic:**

1. Uncomment the block (remove the leading `# `), or add your own. Each top-level
   key (`attitude`, `battery`, …) becomes the ROS topic name `/<cf>/<name>`.
2. Set `frequency` (Hz) — **keep it low**; every block shares the Crazyradio
   bandwidth (see [MOCAP Section 3c](MOCAP.md#3c-trim-drone-log-topics-to-protect-radio-bandwidth)).
3. List the firmware `vars` you want. Discover valid names with:
   ```bash
   ros2 run crazyflie listLogVariables --uri radio://0/80/2M/E7E7E7E701
   ```
4. Rebuild and relaunch:
   ```bash
   ./scripts/build.sh crazyflie     # rebuild after editing the config
   ros2 launch crazyflie launch.py
   ```

**To view a topic** (type is `crazyflie_interfaces/LogDataGeneric`):

```bash
ros2 topic list | grep cf1          # see which log topics exist
ros2 topic echo /cf1/attitude       # live values
ros2 topic hz /cf1/attitude         # confirm it streams at the set rate
```

The `values` array in `LogDataGeneric` is ordered exactly as the `vars` list.
The `default_topics` (`pose`, `status`) are enabled the same way — they're just
predefined names the server understands.

> Tip: enable high-rate blocks (e.g. 50 Hz attitude/kalman) only while actively
> debugging, then comment them out again for normal flight to free up the radio.

## H. Color LED control (Color LED deck)

Applies to drones carrying the bottom-mounted **Color LED deck**
(`bcColorLedBot`; the firmware must expose the `colorLedBot` params). Everything
below drives a single firmware parameter — `colorLedBot.wrgb8888`, a uint32
packed as `0xWWRRGGBB`. White uses the dedicated **W** channel (`0xFF000000`);
an RGB-mixed white looks purplish. **Hardware only** — the `sim` backend
declares no firmware params, so LED commands have no effect there.

### Automatic status behavior (no command needed)

- **Green on connect** — `crazyflie_server` sets full green when a drone
  connects; a drone without the deck just logs
  `No Color LED deck detected — skipping LED set`.
- **Red while a script runs** — `crazyflie_py`'s `Crazyswarm()` sets the deck
  red at startup and restores green on exit via an `atexit` hook, so it
  recovers on normal exit, exceptions, and Ctrl-C.

### Manual control (server must be running)

Both tools accept a color name (case-insensitive) or a number key, and share
the same mapping:

| Key | Color | Hex (WRGB) | Decimal |
|-----|-------|-----------|---------|
| `0` | black (off) | `0x000000` | `0` |
| `1` | green | `0x00FF00` | `65280` |
| `2` | red | `0xFF0000` | `16711680` |
| `3` | yellow | `0xFFFF00` | `16776960` |
| `4` | blue | `0x0000FF` | `255` |
| `5` | purple | `0x9400D3` | `9699539` |
| `9` | white | `0xFF000000` | `4278190080` |

**`scripts/led.sh` — recommended.** Wraps the `ros2 param set` CLI; the
long-running ROS 2 CLI daemon keeps the graph warm, which makes this the most
reliable path. It re-discovers every connected drone's LED param on each color
change, so all drones with the deck switch together.

```bash
./scripts/led.sh yellow     # one-shot by name
./scripts/led.sh 3          # one-shot by number key (= yellow)
./scripts/led.sh 0          # off (same as "black")
./scripts/led.sh            # interactive — press 0-5/9 to switch, q to quit
```

**`ros2 run crazyflie_examples color_led`** — the same control as an rclpy
node (calls `/crazyflie_server/set_parameters` directly; needs the workspace
built and sourced):

```bash
ros2 run crazyflie_examples color_led yellow
ros2 run crazyflie_examples color_led 3
ros2 run crazyflie_examples color_led       # interactive (q or Ctrl-C quits)
```

**Raw CLI** — what both boil down to (decimal value, one drone at a time):

```bash
ros2 param set /crazyflie_server cf1.params.colorLedBot.wrgb8888 16776960   # yellow
```

Auto-discovery of the per-drone LED params requires
`firmware_params: query_all_values_on_connect: True` in `config/server.yaml` —
already set in this repo (it makes connecting slightly slower).

### Standalone cflib demo (server must be STOPPED)

`scripts/color_led_cflib.py` talks to one drone directly over the Crazyradio
with cflib — no ROS involved. It plays a fixed sequence
(green → red → green → blue → white → off) and exits.

```bash
# stop `ros2 launch crazyflie launch.py` first — cflib and the server
# cannot share the radio dongle
python3 scripts/color_led_cflib.py
```

The target URI is hardcoded near the top of the file (marked `# EDIT ME`) —
set it to the drone you want. The script also enables the deck's brightness
correction (`colorLedBot.brightCorr = 1`) on connect.

## Launch arguments (crazyflie launch.py)

| Arg | Values | Meaning |
|-----|--------|---------|
| `backend` | `cpp` \| `cflib` \| `sim` | Hardware (C++/Python) or simulation |
| `mocap` | `True` \| `False` | Start the motion_capture_tracking node (default `True`) |
| `teleop` | `True` \| `False` | Start joystick teleop + joy node (default `True`) |
| `rviz` | `True` \| `False` | Start RViz (default **`True`**; `rviz:=false` to disable) |
| `preflight` | `True` \| `False` | Start the preflight GUI (default **`True`**; see section C) |
| `gui` | `True` \| `False` | Start the swarm GUI (default **`False`**) |
| `foxglove` | `True` \| `False` | Start the foxglove bridge (default `True`; needs `ros-$ROS_DISTRO-foxglove-bridge`) |
| `debug` | `True` \| `False` | Launch the C++ server under gdb (default `False`) |
