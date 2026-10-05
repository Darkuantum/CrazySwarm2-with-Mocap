# CAROUSEL — runcard

Five drones, ~63 s, twelve phases. Fully choreographed — the whole path is
known before takeoff. The original show; `demo_show` is its unchanged starter.

Why any of it is the way it is: [`../SHOW_GUIDE.md`](../SHOW_GUIDE.md).
Shared rules (address scan, E-STOP, LOCKED): [`README.md`](README.md).

> **Ctrl-C does NOT land this show.** `swarm_show` installs no signal handler,
> so `rclpy` tears the ROS context down before the script sees the interrupt
> and the drones keep flying their last command. `constellation_show` and
> `escort_show` take SIGINT back and land; this one does not. **To stop the
> carousel early, use the E-STOP** — and accept the power cycle it costs.
> See [README.md](README.md).

---

## 0. Ground check — no radio, no mocap, no drones

```bash
ros2 run crazyflie_shows plan_show
ros2 run crazyflie_shows plan_show --plot        # writes show_plan.png
```

**This show sits at ~99 % of its separation budget.** A plan that passed before
a position sync can fail after one. Re-run it every time.

## 1. Place the drones and sync

```bash
ros2 launch crazyflie launch.py server:=False    # mocap + GUIs, no radio owner
python3 scripts/sync_initial_positions.py
ros2 run crazyflie_shows plan_show               # re-check
```

Each drone at **its own** yaml position, ≥1 m apart. Wrong-corner placement
means crossing goTo paths — that is the real collision of 2026-08-04.

## 2. Scan, then launch

```bash
# scan every enabled address -- see README.md for the loop
ros2 launch crazyflie_shows show_launch.py
```

Clear the preflight GUI for every drone that will fly.

## 3. Fly

```bash
# sim first, ALWAYS
ros2 launch crazyflie_shows show_launch.py backend:=sim
ros2 run crazyflie_shows swarm_show --ros-args -p use_sim_time:=true

# hardware: dress rehearsal, arms nothing
ros2 run crazyflie_shows swarm_show --ros-args -p dry_run:=true

# hardware: the show
ros2 run crazyflie_shows swarm_show
```

## Arguments — all of them, with defaults

```
dry_run           false      print the full plan and exit without arming
check_placement   true       compare live pose to initial_position
placement_tol     0.25   m   how far off that may be
scale             1.0        shrink/grow every figure about the room centre
arena_radius      1.90   m   plan limit = tested 2.00 - 0.10 tracking margin
ceiling           2.32   m   centre ceiling 2.42 - 0.10 margin
use_sim_time      false      REQUIRED in sim, NEVER on hardware
```

`swarm_show` has **no** `lights` parameter — `constellation_show` and
`escort_show` do.

`scale` changes the geometry — re-run `plan_show` after it.

## Numbers

```
duration        ~63 s wall clock, 60.3 s of motion, 12 phases
ring            1.10 m radius; at n=5 that is 1.29 m between neighbours
altitudes       takeoff 1.00 m, figures 1.00 m, home 0.75 m above each mark
landing         to 0.04 m over 3.5 s (~0.20 m/s descent)
separation      hard floor 0.80 m; plans must hold 0.90 m
room centre     centroid of the drones' own marks (NOT the tracked volume --
                that is what constellation changed to; this show did not)
```

## Abort

| situation | do |
|---|---|
| before arming | Ctrl-C is safe — nothing is flying |
| **in flight** | **E-STOP.** Ctrl-C will not land it (see the box above) |
| after an E-STOP | battery out-and-in on every drone, or nothing will arm |
