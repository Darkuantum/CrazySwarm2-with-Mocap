# CONSTELLATION — runcard

Five drones, 76 s, shape changes on a 120 BPM beat grid with light cues.
Fully choreographed — the whole path is known before takeoff.

Why any of it is the way it is: [`../CONSTELLATION.md`](../CONSTELLATION.md).
Shared rules (address scan, E-STOP, LOCKED): [`README.md`](README.md).

---

## 0. Ground check — no radio, no mocap, no drones

```bash
ros2 run crazyflie_shows plan_constellation       # must print PLAN OK
```

Clearance is enforced in **plan view**: no drone may pass over another. Slots
are assigned by bottleneck distance, not by drone name.

## 1. Place the drones and sync

```bash
ros2 launch crazyflie launch.py server:=False     # mocap + GUIs, no radio owner
python3 scripts/sync_initial_positions.py
ros2 run crazyflie_shows plan_constellation       # re-check: PLAN OK
```

Figures are built at the centre of the **tracked volume**, not at the centroid
of where the drones are parked — so the marks only have to be reachable.

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
ros2 run crazyflie_shows constellation_show --ros-args -p use_sim_time:=true

# hardware: dress rehearsal, arms nothing
ros2 run crazyflie_shows constellation_show --ros-args -p dry_run:=true

# hardware: the show
ros2 run crazyflie_shows constellation_show
```

The upload takes ~55 s, so the show checks the supervisor **before** uploading —
an arming failure used to surface only after the wait.

## Arguments — all of them, with defaults

```
dry_run           false      print the full plan and exit without arming
check_placement   true       compare live pose to initial_position
placement_tol     0.25   m   how far off that may be
lights            true*      LED cues (*forced off under use_sim_time)
bpm               120.0      tempo; re-times EVERY phase and re-runs every check
scale             0.95       shrink/grow every figure about the room centre
arena_radius      1.90   m   plan limit = tested 2.00 - 0.10 tracking margin
ceiling           2.32   m   centre ceiling 2.42 - 0.10 margin
use_sim_time      false      REQUIRED in sim, NEVER on hardware
```

`bpm` and `scale` change the geometry — **re-run `plan_constellation` after
either**, and do not fly a plan that does not print PLAN OK.

## Numbers

```
duration        76 s, every phase boundary on a beat (120 BPM = 0.5 s/beat)
envelope        1.76 m used of the 1.90 m plan limit (2026-10-06 marks)
                  -- and 1.53 m of that is the PARKING, not the figures.
                  Park on the gather ring (R=1.10 at the slot angles) and the
                  same show plans to 1.46 m. Re-run plan_constellation after
                  every sync; this number moves with initial_position.
room centre     [+0.03, +0.26]  -- centre of the tracked volume
separation      hard floor 0.80 m; plans must hold 0.90 m
fleet           5 drones. 6+ does not fit this room.
```

## Abort

| situation | do |
|---|---|
| running, any time | **Ctrl-C lands** — staged abort, it takes SIGINT back from rclpy |
| second Ctrl-C | kills the process outright (handler restores `SIG_DFL`) |
| drone misbehaving | E-STOP — see [README.md](README.md), costs a power cycle |
