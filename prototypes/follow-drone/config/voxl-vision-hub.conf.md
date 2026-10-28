# `voxl-vision-hub.conf` key reference

This file documents every key in `config/voxl-vision-hub.conf`, which is
deployed to `/etc/modalai/voxl-vision-hub.conf` on the VOXL 2. It is set up
for **Offboard trajectory control with Vision Obstacle Avoidance (VOA)
enabled** -- i.e. the mode used when the person-follower is actually flying
the vehicle via voxl-mapper trajectories, not manual/demo modes.

Confidence note: the keys under "MAVLink / localhost link", "VIO health
gating", "Offboard behaviour", and "VOA / obstacle avoidance" below are
corroborated against ModalAI's published docs (docs.modalai.com) and match
the research-verified facts this project's config is built around (VOA
defaults, the 20 Hz send rate, the 36-slice / 0.25-20 m pie geometry, the
14551 localhost port, and the 3.0 s VIO warmup). The exact key names under
"waypoint-mission" and "tag-relocalization" further down are less central to
this project (we don't use those modes) and should be diffed against
`/etc/modalai/voxl-vision-hub.conf` on the actual VOXL 2 the first time you
touch this file on real hardware, since exact schema can drift between SDK
releases -- treat this file as the starting point to diff from, not as a
substitute for checking the installed default.

## MAVLink / localhost link

| Key | Value | Meaning |
|---|---|---|
| `en_localhost_mavlink_udp` | `true` | Opens a **localhost-only** UDP MAVLink socket, separate from the normal GCS telemetry link, so on-target processes (MAVSDK/MAVROS running on the VOXL 2 itself) can talk to PX4 without going over radio. This is what the follow_core supervisor / ROS 2 adapter should connect to. |
| `localhost_udp_port_number` | `14551` | Port for that socket. `14550` is reserved for the off-board GCS (QGroundControl) link; `14551` keeps the on-target autonomy link separate so QGC and the autonomy stack never fight over the same port. |

## VIO health gating

| Key | Value | Meaning |
|---|---|---|
| `en_vio` | `true` | Forward qVIO estimates to PX4 as MAVLink `ODOMETRY` messages. Required for any position control at all (Position, Offboard, or trajectory mode) since the Starling has no GPS indoors. |
| `vio_pipe` | `"qvio"` | Primary VIO source: ModalAI's own qVIO pipe. |
| `secondary_vio_pipe` | `""` | Fallback VIO pipe (e.g. `"ov"` for OpenVINS) if qVIO drops. Left empty deliberately: this project standardizes on qVIO only, and an unconfigured fallback should not silently become the primary. |
| `en_reset_vio_if_initialized_inverted` | `true` | Auto-resets VIO if it initializes upside-down or badly off-level (a known qVIO failure mode on power-up). Cheap insurance, no reason to disable. |
| `vio_warmup_s` | `3.0` | **Gate**: voxl-vision-hub will not hand odometry to PX4 as "good" until qVIO has held a healthy lock for this many seconds. This is why `docs/BRINGUP.md` insists on visually confirming qVIO is healthy and settled *before* starting voxl-mapper or attempting flight -- a fresh/jittery VIO lock will not clear this warmup window and downstream nodes that assume valid odometry will misbehave. |
| `send_odom_while_failed` | `true` | Keep sending odometry (with quality `-1`) to PX4 during a VIO failure so PX4's EKF2 can dead-reckon briefly rather than getting no updates at all. Does not bypass the warmup gate above for re-establishing "good" status. |

## Offboard behaviour

| Key | Value | Meaning |
|---|---|---|
| `en_hitl` | `false` | Hardware-in-the-loop simulation mode (VIO sourced from a simulator pipe instead of the real camera). Off for real flight. |
| `offboard_mode` | `"trajectory"` | **The key setting for this project.** Selects which autonomous behavior voxl-vision-hub drives when PX4 is put into Offboard mode. `"trajectory"` means voxl-vision-hub expects **voxl-mapper** to be running and publishing to the `plan_msgs` pipe, and it relays voxl-mapper's planned, obstacle-checked trajectory to PX4. This is a hard dependency: with no producer on `plan_msgs`, Offboard mode has nothing to fly. Other enum values (`"figure_eight"`, `"follow_tag"`, `"vfc"`, `"backtrack"`, `"wps"`, `"off"`) are ModalAI demo/utility modes not used by this project. |
| `follow_tag_id` | `0` | AprilTag ID used only by `"follow_tag"` mode. Unused here; left at default since `offboard_mode` is `"trajectory"`. |
| `figure_eight_move_home` | `false` | Only relevant to `"figure_eight"` demo mode. Unused here. |
| `robot_radius` | `0.30` | Collision radius (metres) used by voxl-vision-hub's own trajectory monitor as a second check on top of voxl-mapper's planning radius. Set to roughly the Starling 2 Max's prop-tip radius plus a small margin; **measure your actual prop diameter and confirm** this is ≥ half the prop span plus ~5 cm. |
| `collision_sampling_dt` | `0.10` | Sampling step (metres, despite the name -- this is a spatial step along the trajectory, not a time step) used when checking the incoming trajectory for collisions against the map. Left at the ModalAI default. |
| `max_lookahead_distance` | `1.0` | How far ahead (metres) along the trajectory the collision monitor looks before allowing more of it through. Left at default; small indoor room does not need a longer look-ahead. |

## Waypoint-mission mode (`wps_*`) -- not used by this project

These only matter when `offboard_mode` is `"wps"`. Listed for completeness /
future reference since the schema includes them; left at ModalAI defaults
except `figure_eight_move_home`/`wps_vfc_mission`, which are explicitly
disabled so an operator can't accidentally trigger a demo mode instead of the
real follower.

| Key | Value | Meaning |
|---|---|---|
| `wps_move_home` | `true` | Shift a waypoint mission to start at the vehicle's current position. |
| `wps_stride` | `0.0` | Interpolate intermediate waypoints every N metres; `0` disables. |
| `wps_timeout` | `0.0` | Hover pause at each waypoint, seconds; `0` disables. |
| `wps_damp` | `1.0` | Deceleration damping factor approaching a waypoint. |
| `wps_vfc_mission` | `false` | Use ModalAI's VFC controller instead of raw PX4 setpoints for a waypoint mission. |
| `wps_vfc_mission_loop` | `false` | Loop the waypoint mission. |
| `wps_vfc_mission_cruise_speed` | `1.0` | Cruise speed (m/s) for VFC-driven waypoint missions. |

## RC tracking trigger -- not used by this project

| Key | Value | Meaning |
|---|---|---|
| `tracking_trigger_ch` | `12` | RC channel used to arm/latch/clear ModalAI's built-in object tracking. This project's operator commands come from the tablet app and gesture band (see `follow_core`), not this RC-channel mechanism, so this is left at its default and effectively unused, but must not collide with a channel your RC transmitter mapping actually uses for something else. |
| `tracking_trigger_ch_thresh_temp` | `1200` | PWM (µs) threshold for a momentary tracking trigger. |
| `tracking_trigger_ch_thresh_new` | `1800` | PWM (µs) threshold to latch a new tracking target. |

## Tag-relocalization -- not used by this project

| Key | Value | Meaning |
|---|---|---|
| `en_tag_fixed_frame` | `false` | AprilTag-based drift correction / fixed-frame relocalization. Not part of this project's localization plan (hybrid UWB + vision to the operator, not fiducial tags), left disabled. |
| `fixed_frame_filter_len` | `5` | Filter length for fixed-frame pose estimates, only relevant if the above is enabled. |
| `en_transform_mavlink_pos_setpoints_from_fixed_frame` | `false` | Whether incoming position setpoints are interpreted in the tag-fixed frame. Left disabled to match. |

## VOA -- Vision Obstacle Avoidance

This is the second key section for this project, alongside `offboard_mode`.
VOA turns depth data (ToF + stereo) into a MAVLink `OBSTACLE_DISTANCE`
message PX4 can consume, and is also what feeds voxl-mapper's own obstacle
picture at the raw-sensor level.

| Key | Value | Meaning |
|---|---|---|
| `en_voa` | `true` | Master enable for the VOA pipeline. |
| `voa_upper_bound_m` | `-0.15` | Upper vertical bound (metres, vehicle frame, negative = above) of the slab of space VOA considers for obstacles -- excludes points far above the vehicle (e.g. a ceiling truss) from counting as an "obstacle" in the horizontal pie. |
| `voa_lower_bound_m` | `0.15` | Lower vertical bound -- excludes points from the floor immediately under the vehicle. |
| `voa_voa_memory_s` | `1.0` | How long (seconds) an observed obstacle point persists in VOA's internal memory after last being seen, so a brief dropout in one sensor doesn't instantly erase known obstacles from the picture. |
| `voa_max_pc_per_fusion` | `100` | Cap on point clouds fused into one VOA update cycle -- a compute-load bound. |
| `voa_pie_max_dist_m` | `20.0` | Maximum range of the pie-slice obstacle histogram sent to PX4. Research-verified default; left as-is -- in an indoor room this is far beyond the room's extent, which is fine (it just means "no obstacle reported" beyond room walls). |
| `voa_pie_min_dist_m` | `0.25` | Minimum range of the pie-slice histogram. Research-verified default. Points closer than this (e.g. the vehicle's own frame in the ToF field of view) are excluded. |
| `voa_pie_slices` | `36` | Angular resolution of the obstacle pie: 36 slices = 10° per slice. Research-verified default. |
| `voa_pie_bin_depth_m` | `0.15` | Radial bin size used when discretizing points into the pie histogram. |
| `voa_pie_threshold` | `3` | Minimum point count in a bin before it's marked "occupied" -- filters single-point noise. |
| `voa_pie_under_trim_m` | `1.0` | Distance directly under the vehicle that's trimmed out of consideration (e.g. so the floor itself, seen by a downward-canted sensor, doesn't register as a forward obstacle). |
| `voa_send_rate_hz` | `20.0` | Rate VOA data is sent to PX4. Research-verified default. |

### `voa_inputs` -- sensor sources feeding VOA

Configured for the Starling 2 Max's **C29 camera config (stereo + ToF)**:
one ToF entry and one stereo depth-from-stereo (DFS) point-cloud entry.

| Field | ToF entry | Stereo entry | Meaning |
|---|---|---|---|
| `enabled` | `true` | `true` | Both sensors feed VOA. |
| `type` | `"tof"` | `"point_cloud"` | Sensor modality. |
| `input_pipe` | `"tof"` | `"stereo_front_pc"` | MPA pipe name VOA reads from. **Confirm the exact pipe name against `voxl-list-pipes` on the actual vehicle** -- ModalAI's stereo/DFS pipe naming has varied across SDK releases and camera configs. |
| `max_depth` | `4.0 m` | `8.0 m` | Beyond this range, points from that sensor are ignored. ToF is rated for shorter range than stereo; matches the ToF's practical indoor range and the room's size for stereo. |
| `min_depth` | `0.25 m` | `0.30 m` | Below this range, points are ignored (too close to trust / sensor's own minimum range). |
| `cell_size` | `0.10 m` | `0.10 m` | Matches `voxl-mapper.conf`'s `voxel_size` (see `config/voxl-mapper.conf.md`) so the two don't disagree on map resolution. |
| `threshold` | `3` | `3` | Minimum points before a cell counts as occupied for this sensor. |
| `x_fov_deg` / `y_fov_deg` | `90 / 90` | `90 / 65` | Sensor field of view. **Confirm against the actual C29 datasheet/calibration** -- these are the commonly quoted ballpark FOVs for ModalAI's ToF and stereo-pair modules respectively, not a measured value for this specific unit. |
| `conf_cutoff` | `0.5` | `0.5` | Minimum per-point confidence to include. Mid-point default; tighten (raise) if VOA reports phantom obstacles from noisy returns during bring-up testing. |

## Things to verify on real hardware

- Diff this whole file against the as-shipped `/etc/modalai/voxl-vision-hub.conf`
  on the actual Starling 2 Max before first flight -- SDK versions can add,
  rename, or change the default of keys.
- Confirm `voa_inputs[*].input_pipe` names with `voxl-list-pipes` once
  voxl-mapper and the camera server are running (see `docs/BRINGUP.md`).
- Confirm `robot_radius` against the vehicle's actual prop-tip radius.
