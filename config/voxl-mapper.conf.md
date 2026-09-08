# `voxl-mapper.conf` key reference

This file documents every key in `config/voxl-mapper.conf`, deployed to
`/etc/modalai/voxl-mapper.conf` on the VOXL 2. Tuned for **indoor mapping of
a single room** (the netted mocap volume this project flies in), where the
priorities are: keep compute load bounded on the QRB5165's sensors DSP, and
keep the ESDF planning radius large enough to matter but not so large it
wastes compute mapping distances the room doesn't have.

Confidence note: `voxel_size` and `esdf_max_distance` are the two
research-verified parameters for this project (defaults `0.10 m` /
`2.0 m`, and "reducing voxel size greatly increases compute load" -- see
`docs/BRINGUP.md` and `docs/SAFETY.md` for where that fact matters
operationally). The remaining keys (ToF/depth-pipe wiring, ESDF sphere
radii, RRT*, and LOCO trajectory-optimizer weights) are corroborated from
ModalAI's public docs and example configs for ToF-equipped vehicles, but
**voxl-mapper's schema and defaults have shifted between SDK/model
generations** (e.g. published Seeker-drone examples show `voxel_size: 0.2`
/ `esdf_max_distance: 4` where this project's brief specifies `0.10` /
`2.0`) -- diff this file against the as-shipped default on the actual
Starling 2 Max before first flight, per `docs/BRINGUP.md`.

`voxl-mapper` hard-depends on **voxl-vision-hub** and a healthy **VIO**
lock -- it has nothing to map or plan against without both (see
`docs/BRINGUP.md`, which verifies qVIO health before ever starting
voxl-mapper).

## Sensor inputs

| Key | Value | Meaning |
|---|---|---|
| `tof_enable` | `true` | Enable the Time-of-Flight depth sensor as a mapping input. The Starling 2 Max's C29 camera config includes a ToF sensor, so this should be on. |
| `tof_pipe` | `"/run/mpa/tof"` | MPA pipe the ToF publishes range data on. **Confirm with `voxl-list-pipes`** on the actual vehicle -- pipe naming can vary by SDK build. |
| `tof_rate` | `10` (Hz) | Rate voxl-mapper consumes ToF frames at. ToF sensors on this class of hardware typically run in the ~5-15 Hz range; 10 Hz is a reasonable indoor default and matches typical ModalAI example configs for ToF-equipped vehicles. |
| `depth_pipe_0` / `depth_pipe_0_enable` | `"stereo_front_pc"` / `true` | Primary stereo depth-from-stereo (DFS) point-cloud pipe, enabled. The C29 config's forward stereo pair is the main long-range mapping input (ToF is short-range and narrow-FOV by comparison). |
| `depth_pipe_1` / `depth_pipe_1_enable` | `"stereo_rear_pc"` / `false` | Rear stereo pipe slot, present in the schema but disabled: this project's follow geometry keeps the operator ahead of or beside the vehicle (see `follow_core.FollowParams.follow_angle_rad`), so rear obstacle mapping is not needed for the committed scope. Enable it if the vehicle is expected to ever back away from an obstacle. |
| `depth_pipe_2` / `depth_pipe_3` (+ `_enable`) | `""` / `false` | Unused pipe slots (the schema supports up to 4 depth sources). Left empty/disabled. |

## Mapping resolution

| Key | Value | Meaning |
|---|---|---|
| `voxel_size` | `0.10 m` | **Research-verified default.** Side length of one voxel in the occupancy/TSDF map. Left at the stock default rather than reduced further: the fact this project's config is built around is that *reducing voxel size greatly increases compute load* on the sensors DSP, and 0.10 m is already fine enough to resolve a person and typical room furniture. Do not lower this without profiling CPU headroom first (`docs/BRINGUP.md` / `docs/SAFETY.md` reference this trade-off). |
| `robot_radius` | `0.30 m` | Collision radius used by the planner, matching the value used in `config/voxl-vision-hub.conf`'s trajectory monitor so the two components agree on how much clearance the vehicle needs. **Confirm against the Starling 2 Max's actual prop-tip radius plus margin.** |

## ESDF (Euclidean Signed Distance Field)

| Key | Value | Meaning |
|---|---|---|
| `esdf_max_distance` | `2.0 m` | **Research-verified default.** How far from any surface the ESDF computes a distance value. ModalAI's guidance is that mapping-only use cases can set this very small, but this project uses voxl-mapper's *planning* output (the follower hands voxl-mapper a goal and it must route around obstacles), so it is kept at a value comfortably larger than `robot_radius` (0.30 m) as required for planning to work at all, while still being small enough for a single room and bounded compute. |
| `esdf_min_distance` | `0.05 m` | Floor on the computed distance value near a surface -- avoids numerical blow-up exactly at an occupied voxel boundary. |
| `esdf_default_distance` | `2.0 m` | Default distance value assigned to unobserved space, matched to `esdf_max_distance` so unmapped regions read as "far from anything," not falsely "close to an obstacle." |
| `esdf_inner_sphere_radius` | `0.20 m` | Inner radius of the two-sphere approximation voxl-mapper's planner uses to cheaply bound collision cost near the vehicle. Smaller than `robot_radius` is not correct in general, but this value follows ModalAI's own example ratio between inner sphere and robot radius; **treat as a starting point to validate against real planned paths during bring-up**, not a guaranteed-safe number on its own -- the LOCO collision cost weight (below) is what actually keeps the vehicle away from surfaces. |
| `esdf_outer_sphere_radius` | `0.60 m` | Outer radius of that same two-sphere model -- the distance beyond which the planner considers a point to contribute negligible collision cost. |

## RRT* global planner

| Key | Value | Meaning |
|---|---|---|
| `rrt_min_distance` | `0.10 m` | Minimum edge length in the RRT* tree. Lowered from the commonly-quoted `0.2 m` example default to match this project's finer `voxel_size` and the fact that a single small room needs the planner able to find paths through tighter gaps than a large open Seeker-drone deployment would. |
| `rrt_max_runtime_nanoseconds` | `500000000` (0.5 s) | Wall-clock budget per planning call. Kept well under a second so a follow-trajectory replan (triggered as the operator moves) doesn't stall the control loop noticeably. |
| `rrt_use_first_solution` | `false` | Let RRT* keep improving the path within its time budget rather than returning the very first feasible (often jagged) path found -- worth the extra planning time in a small room where a short, smooth path matters more than raw speed. |
| `rrt_treat_unknown_as_occupied` | `true` | Treat unmapped space as an obstacle rather than free space. The conservative, safety-preferred choice: a person-following vehicle in a room with a person in it must never plan through space it hasn't actually observed. |

## LOCO trajectory optimizer

Smooths the RRT* path into a dynamically-flyable polynomial trajectory
before handing it to `voxl-vision-hub` (`offboard_mode: "trajectory"`, see
`config/voxl-vision-hub.conf`).

| Key | Value | Meaning |
|---|---|---|
| `loco_num_segments` | `5` | Number of polynomial segments the trajectory is broken into. ModalAI default; adequate for the short paths a single room requires. |
| `loco_poly_degree` | `4` | Degree of each segment's polynomial. ModalAI default -- high enough for smooth, jerk-bounded motion. |
| `loco_smoothness_cost_weight` | `2.5` | Weight on trajectory smoothness in the optimizer's cost function. |
| `loco_collision_cost_weight` | `18.0` | Weight on staying clear of mapped obstacles, weighted well above smoothness so the optimizer prefers a slightly less smooth path over one that cuts close to a wall or the person -- appropriate given a person is in the same small volume. |

## Things to verify on real hardware

- Diff this whole file against the as-shipped `/etc/modalai/voxl-mapper.conf`
  before first flight -- confirmed schema drift exists between SDK/model
  generations (see the Seeker-drone example values noted above).
- Confirm `tof_pipe` / `depth_pipe_0` pipe names with `voxl-list-pipes` once
  the camera server is running (`docs/BRINGUP.md`).
- Profile CPU load with `voxel_size: 0.10` on the actual sensors DSP before
  relying on it in a live test; drop to `0.15-0.20` if the mapper can't keep
  up, rather than raising it blind.
