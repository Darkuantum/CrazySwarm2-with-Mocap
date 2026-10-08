# ESCORT — runcard

Three defenders ring a VIP; a fourth drone attacks; the ring rotates so a
defender sits on the threat bearing. **Reactive, not choreographed** — the
only show here whose path is not known before takeoff.

Why any of it is the way it is: [`src/crazyswarm2/crazyflie_shows/ESCORT.md`](../src/crazyswarm2/crazyflie_shows/ESCORT.md).
Shared rules (address scan, E-STOP, LOCKED): [`README.md`](README.md).

---

## 0. Ground checks — no radio, no mocap, no drones

```bash
ros2 run crazyflie_shows plan_escort              # must end "static case OK"
ros2 run crazyflie_shows plan_escort --marks      # must end "MARKS OK"
ros2 run crazyflie_shows plan_escort --sweep      # how fast the VIP may move
ros2 run crazyflie_shows plan_escort --plot /tmp/escort.png
```

`plan_escort` verifies the *same* objects the flight commands, so a pass here
is worth something. It currently reports **"the walking case is NOT cleared"** —
that is expected and refers to a 0.30 m/s walking VIP. The static case, which
is what you fly, passes.

## 1. Place the drones and sync

Mocap and the GUIs, **no server** — nothing owns the radio, nothing is armable:

```bash
ros2 launch crazyflie launch.py server:=False
python3 scripts/place_drones.py          # marks in RViz on /escort/marks
python3 scripts/sync_initial_positions.py
ros2 run crazyflie_shows plan_escort --marks     # re-check: "MARKS OK"
```

`place_drones.py` re-reads the config while running, so edits move the marks
without a restart. Stand each drone on its mark (green at 10 cm), then sync —
the yaml must record where they **are**, not where they were meant to go.

## 2. Scan, then launch

```bash
# scan every enabled address -- see README.md for the loop
ros2 launch crazyflie launch.py
```

Clear the preflight GUI for every drone that will fly.

## 3. Fly

```bash
# rehearsal: prints the plan, arms nothing
ros2 run crazyflie_shows escort_show --ros-args -p dry_run:=true

# the show, operator-paced (recommended)
ros2 run crazyflie_shows escort_show --ros-args \
    -p vip_mode:=point -p adversary:=scripted -p paced:=true
```

In **sim**, add `-p use_sim_time:=true` to this *and* to any teleop. On
hardware, never.

## Arguments — all of them, with defaults

```
dry_run           false      verify and report, then exit
paced             false      Enter gates arming AND every leg; 'q' lands
check_placement   true       compare live pose to initial_position
placement_tol     0.25   m   how far off that may be
lights            true*      LED cues (*forced off under use_sim_time)
duration          <script>   s; the script's own length, or 120 when a human
                             drives the adversary
use_sim_time      false      REQUIRED in sim, NEVER on hardware

vip_mode          point      point | mocap | manual
vip_point         <mark>     m, static VIP; default room centre + vip_offset
vip_name          operator   rigid body to escort      (vip_mode:=mocap)
vip_airborne      <auto>     default true only when vip_mode:=mocap
adversary         scripted   scripted | external | manual | none
adversary_name    adv        its rigid body            (adversary:=external)
defenders         ''         e.g. cf1,cf2,cf3 -- comma-separated STRING
adversary_drone   ''         which drone attacks; default = the 4th enabled

# EscortConfig overrides -- each re-verified before takeoff, refused if it
# does not check out
ring_radius       1.0    m   also the chord the closing wall uses
height            1.2    m
v_max             0.6    m/s
phase_rate        0.3    rad/s
alert_radius      2.10   m   adversary-to-VIP distance that ENGAGES blocking
release_radius    2.22   m   the larger one that disengages
min_vip_dist      1.0    m   hard floor, defender to VIP (= ring_radius)
rate_hz           20.0       setpoint stream rate per drone
vip_height_offset 0.3    m   ring altitude relative to an airborne VIP
```

`probe < alert < release < retreat` must hold. `plan_escort` refuses if it
does not — run it after changing either radius.

## Teleop (no DJI, nobody in the room)

Separate terminal, **needs a real TTY**. Publishes a virtual target; commands
no drone.

```bash
# you drive the attacker, VIP is a fixed point
ros2 run crazyflie_shows escort_show   --ros-args -p vip_mode:=point -p adversary:=manual
ros2 run crazyflie_shows escort_teleop --ros-args -p target:=adversary

# you drive the VIP (DJI stand-in), attacker runs its proven script
ros2 run crazyflie_shows escort_show   --ros-args -p vip_mode:=manual -p adversary:=scripted
ros2 run crazyflie_shows escort_teleop --ros-args -p target:=vip
```

`wasd`/arrows in plan view, `q`/`e` altitude (adversary only), space stop,
`c` re-centre, `x` quit. Hold the key — velocity decays 0.35 s after the last
keystroke.

**Start the teleop, then touch nothing until `ESCORT LIVE` appears.** The show
reads the target before takeoff and checks the gather against it; a target
already moving makes the gather refuse (`gather legs close to 0.00 m`) and land.

## Numbers

```
ring            1.00 m radius, 1.20 m altitude, 3 defenders 120 deg apart
                (+0.30 m above the DJI when it is airborne)

MARKS AND THE ARENA CENTRE ARE NOT WRITTEN HERE ON PURPOSE. Print them:
    ros2 run crazyflie_shows plan_escort --marks
Every coordinate in this block was in the pre-2026-10-08 world frame. Motive
was recalibrated that day and the frame rotated ~91.6 deg, so the numbers that
used to sit here became wrong in a way that still LOOKED plausible -- which is
exactly how a drone gets stood on the wrong mark. The marks also move whenever
ring_radius or vip_offset changes. The planner reads the live config; prose
does not.
geofence        VIP may stray 0.90 m from room centre (arena 1.90 - ring 1.00),
                then the ring HOLDS and all three defenders go CYAN. It
                resumes once the DJI is back within 0.65 m AND inside the
                parked ring -- so fly it to the MIDDLE, not just back in.
VIP speed       0.30 m/s worst case; --sweep clears the run only to 0.20 m/s
```

Marks move whenever `ring_radius` or `vip_offset` does — **print them, do not
copy them from here.**

## Reviewing a run against the video

Every line the show prints carries show time **and** wall clock, and the
`ESCORT LIVE` banner prints `t0`, so `t+` converts to a timecode without
guessing. On exit — including aborts — the show prints a table:

```
  GEOFENCE EPISODES (2)
    #   show time   wall clock   held
    1   t+  12.3s   14:32:07       6.4 s
    2   t+  48.9s   14:32:44      10.2 s
    the defenders were CYAN for exactly these windows
```

That is the table to lay against the recording. The LED tells the operator the
fence is on *now*; this is what tells you *when*, afterwards — and matching the
colour changes to fence events was impossible after the 2026-10-07 run because
nothing printed an absolute time.

For an independent check, run the monitor in a second terminal. It recomputes
the same fence predicate from `/poses` alone and commands nothing, so its
timeline can be diffed against the show's own claim:

```bash
python3 scripts/watch_escort.py --log ~/escort-$(date +%F-%H%M).jsonl
```

It prints a hold table of its own at exit, and the JSONL carries `wall` and
`clock` on every entry.

## Abort

| situation | do |
|---|---|
| paced, between legs | `q` + Enter — lands normally |
| running, any time | **Ctrl-C lands** (it takes SIGINT back from rclpy) |
| drone misbehaving | E-STOP — see [README.md](README.md), costs a power cycle |
| teleop quits | VIP stale 0.4 s → hold, 2.0 s → land. Adversary stale → freezes |
