# MOCAP — OptiTrack setup, rigid bodies, and frequency tuning

This stack uses an OptiTrack camera system running **Motive** (on the Windows
host) to localize the drones. Motive streams pose data over NatNet (multicast)
to `motion_capture_tracking` — started by `launch.py` — which publishes
`/poses` for the swarm server. The open `natnet_ros2` + `pose_bridge.py` path
is an alternative, not the default (see [RUNNING Section B](RUNNING.md#b-hardware-flight-with-optitrack)).

Four things must be right before flight: a **good calibration**, a **clean rigid
body per drone**, the **rigid body's forward axis on +X** (Section 2), and a
**streaming rate matched to your radio bandwidth**.

> Config files referred to below as `config/<name>.yaml` live at
> `src/crazyswarm2/crazyflie/config/<name>.yaml`.

---

## 1. Camera calibration (brief)

Full procedure: <https://docs.optitrack.com/motive/calibration>.

1. **Mask extraneous reflections.** In Motive's *Calibration* pane, with no
   markers in the volume, use **Mask Visible** so the cameras ignore fixed
   reflective sources (shiny floor, window frames, LED fixtures).
2. **Wand the volume.** Select **Start Wanding** and wave the calibration wand
   through the whole flight volume, covering all heights and corners until each
   camera collects enough samples (the per-camera bars fill up).
3. **Calculate** and apply the result; aim for an "Exceptional" / "Excellent"
   mean error.
4. **Set the ground plane** with the calibration square, defining the origin and
   axes. Use **Z-up** so the orientation matches what crazyswarm2 expects.

> **Reading the camera LED — is a camera seeing noise?**
> Each OptiTrack camera has a status LED. When a camera is **flashing white**, it
> is detecting reflections / marker-like blobs in its view. Before flight you want
> the cameras *quiet* (no flashing) when the volume is empty — a camera that keeps
> flashing white with nothing in the volume is seeing stray reflections or noise
> and should be re-masked (step 1) or the offending reflector removed. During
> flight, flashing white simply means it sees the drone markers, which is expected.

## 2. Defining rigid bodies

Full procedure: <https://docs.optitrack.com/motive/rigid-body-tracking>.

Each drone needs its own rigid body so Motive streams one pose per drone:

1. Place the drone in the volume so its markers (or the single active marker /
   mocap-deck pattern) are visible. The markers should appear as a small cluster
   in the *Perspective* view. **Point the drone's forward axis along global +X
   before creating the body** — see the orientation rule below.
2. Select those markers, right-click → **Rigid Body → Create From Selected
   Markers** (or press **Ctrl+T**).
3. In the rigid body's properties, give it a **name that matches everything
   downstream** — the same string used in:
   - `config/crazyflies.yaml` (the robot key, e.g. `cf1`),
   - `pose_bridge.py` `DRONES` (**alternative natnet_ros2 path only** — it
     reads the roster from `crazyflies.yaml` since 2026-10-06, so there is
     nothing to keep in sync; before that it carried a hardcoded
     `['cf1', 'cf2']` that had been wrong for months),
   - the `/<name>/pose` topic published by `natnet_ros2`.
4. Enable **streaming IDs / names** and make sure orientation looks stable (the
   rigid body's axes shouldn't jitter or flip). Keep marker patterns
   **asymmetric and distinct between drones** so Motive doesn't swap their IDs.

> **Orientation rule.** The rigid body's zero orientation is captured at
> creation, so create it with the drone's **forward axis aligned to global
> +X**. Otherwise the onboard yaw inherits a constant offset: mocap *position*
> is force-fused into the Kalman (`locSrv.extPosStdDev = 1e-3`), so position
> looks perfect (~1 mm) even with a rotated body — the offset is invisible at
> rest and becomes a fly-away in flight. The preflight GUI catches this (red
> misalignment banner / dashed `err.yaw`, see
> [RUNNING Section C](RUNNING.md#c-preflight-gui-preflight_kalman_plotterpy)); RViz
> shows it as rotated axes between the `<cf>` and `<cf>_mocap` frames. If in
> doubt, delete the rigid body, re-orient the drone, and recreate it.

> The marker geometry in `config/motion_capture.yaml`
> (`marker_configurations`) must match the physical marker layout on the drone
> (`default_single_marker`, `mocap_deck`, etc.).

## 2b. Setting `initial_position` from `/poses`

Whenever the drones' floor placement changes, update each drone's
`initial_position` in `config/crazyflies.yaml` from **mocap truth**. Use the
script; it carries the safety refusals that a hand edit does not:

1. **Place** the drones on the floor where they will take off (the show
   planners' `--marks` output, or `scripts/place_drones.py`, says where).
2. **Bring up mocap WITHOUT the server**, so there is no radio owner:
   ```bash
   ros2 launch crazyflie launch.py server:=False
   ```
3. **Preview, then apply:**
   ```bash
   python3 scripts/sync_initial_positions.py --dry-run     # shows the diff, writes nothing
   python3 scripts/sync_initial_positions.py               # prompts; --yes to skip the prompt
   ```
   It samples `/poses`, matches each drone to the rigid body of the same name
   and rewrites only the `initial_position` triples, comments preserved. It
   writes a mark for **every drone mocap streams, enabled or not** (so
   switching fleets between shows never carries a stale mark;
   `--enabled-only` restores the old behaviour). It **refuses** when an enabled
   drone is not streamed, is moving (>10 mm spread over the window), sits above
   0.5 m, or when two drones are <1 m apart; `--force` overrides and `--with-z`
   writes the measured z instead of keeping the yaml's.
4. **Restart the server** — the yaml is read **only at launch**; an edit while
   the server runs changes nothing.
5. **Re-run every show planner** (`plan_show`, `plan_constellation`,
   `plan_escort`): the plan is a function of the marks.

Manual equivalent, if you must: read x/y from
`ros2 topic echo /poses | grep -A5 -- '- name: cf1$'` and edit by hand.

Rules — each one is load-bearing:

- **NEVER copy from `/cfX/pose`.** That is the onboard estimate, which is
  **seeded by the yaml** — copying it back is circular and just launders
  whatever stale value was there. `/poses` is the mocap ground truth.
- **Every pair of enabled drones ≥ 1 m apart.** The multi-drone demos fly the
  same relative trajectory on every drone, so the flight **preserves the start
  separation** — the spacing on the floor is the spacing in the air.
- **Each drone sits at ITS OWN yaml position.** The flight scripts compute
  **absolute** `goTo` targets from `initial_position` (return-home =
  `initial_position` + height), so two drones swapped between corners means
  the return-home / formation `goTo` paths **cross** — this caused a **real
  mid-air collision on 2026-08-04**. Match names on the floor to names in the
  yaml before every flight.

## 2c. The world frame, the measured arena, and what a recalibration breaks

Everything the shows and the planners know about the room is in
`src/crazyswarm2/crazyflie_shows/crazyflie_shows/safety.py` (print it, do not
copy it: `python3 -c "from crazyflie_shows import safety as s; print(s.ARENA_CENTRE, s.ARENA_RADIUS_TESTED, s.ARENA_RADIUS_PLAN, s.ARENA_RADIUS_LOST)"`).

- **The frame is whatever Motive's ground plane defines.** Z is up (a ground
  plane that is not Z-up must be recalibrated) and each drone's nose must lie
  along global **+X** when its rigid body is created — see section 2. Look at
  the axis triad in Motive, not at your memory of it. `scripts/check_mocap_yaw.py`
  prints each body's own X axis in the world live (mocap up, server not running)
  while you square the drones.
- **The arena was surveyed 2026-10-01** by carrying a tracked body and by
  flying (`scripts/measure_arena.py`, `scripts/arena_flight_sweep.py`; raw
  data under `data/arena/`). The tracked volume is anisotropic, roughly a
  truncated cone; `ARENA_RADIUS_TESTED` is where flight was clean,
  `ARENA_RADIUS_LOST` is where a flying drone actually lost tracking, and
  `ARENA_RADIUS_PLAN` is tested minus the tracking margin, the number a plan
  must stay inside.
- **Motive was recalibrated 2026-10-08, and the frame moved.** The room, the
  cameras and the drones did not. The old-to-new transform was fitted on three
  undisturbed drones (rotate +91.59 deg, translate [+0.043, -0.098] m, max
  residual 0.0159 m; their pairwise distances still agreed with the yaml to
  <= 21 mm, which is what shows the frame moved and not the drones). `ARENA_CENTRE`
  was carried across; the radii are invariant. **A transformed survey is not a
  survey:** re-run `scripts/arena_flight_sweep.py` before trusting the edges.
- **After any Motive recalibration or rigid-body rename:** do not fly; re-sync the
  marks (2b); transform every world-frame literal in every show *by its kind*
  (a point is rotated and translated, an offset is rotated, an absolute bearing
  has the angle added) — the full table, and the escort's three separate
  mistakes, are in [WRITING-A-SHOW §5.6](WRITING-A-SHOW.md); re-run the planners.

## 3. Frequency / bandwidth tuning (240 → 50 Hz)

By default an OptiTrack system may stream at **240 Hz**. That is far more than the
Crazyflie needs and it loads the Crazyradio link, because **every mocap pose is
forwarded to the drone over the radio** as an external position update. Drop the
rate to **50 Hz**, which is plenty for stable position hold, and trim the drone
log topics so the 2 Mbit/s radio is not maxed out.

### 3a. Lower the streaming rate in Motive

1. **View → Settings → Streaming** (or the *Data Streaming* pane).
2. Make sure **Broadcast Frame Data** is enabled and the NatNet streaming engine
   is on (turn off VRPN/Trackd if unused).
3. Set the **data signal / transmission type** to **Multicast** (not Unicast).
   The vendored `motion_capture_tracking` accepts the multicast stream (and
   Motive's **Broadcast Frame Data** variant, which is what this rig actually
   sends); it does **not** support Unicast
   (this repo uses `type: "optitrack"`, the open parser, in
   `config/motion_capture.yaml` — see Section 5 for why). The
   transmission type is read **once at connect** — after changing it in Motive,
   fully restart the launch (and check for leftover frozen mocap processes
   first, see [TROUBLESHOOTING](TROUBLESHOOTING.md#mocap-pipeline)). See Section 4
   for what multicast vs unicast actually means.
4. Set the **point cloud / camera rate** to **50 Hz** (Motive *Settings →
   Cameras → Rate*, or the system rate). The streaming rate follows the camera
   frame rate, and must match `poses.qos.deadline: 50.0` in
   `config/motion_capture.yaml`.

### 3b. Match the ROS side to 50 Hz

These are already set in this repo — keep them consistent if you change the rate:

- **`pose_bridge.py`** → `PUBLISH_HZ = 50.0` (was 240). The publish timer and QoS
  deadline both derive from this. Publishing faster than Motive streams only
  resends stale poses.
- **`config/motion_capture.yaml`** → `topics.poses.qos.deadline: 50.0` so the QoS
  deadline corresponds to the 50 Hz period.

### 3c. Trim drone log topics to protect radio bandwidth

In `config/crazyflies.yaml`, the `all.firmware_logging` block controls how much
telemetry each drone streams back over the radio. High log rates compete with the
mocap pose uplink and the control setpoints for the same 2 Mbit/s. The values in
this repo are intentionally low:

```yaml
all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10   # Hz   (was higher)
      status:
        frequency: 1    # Hz
    custom_topics:
      kalman_preflight:
        frequency: 5    # Hz — feeds the preflight GUI (RUNNING Section C); keep enabled
        vars: [kalman.stateX, kalman.stateY, kalman.stateZ, motion.deltaX,
               motion.deltaY, range.zrange, stateEstimateZ.vx, stateEstimateZ.vy]
```

Guidelines:

- **Drones on the same URI channel share one radio's bandwidth.** This rig
  currently runs **single-dongle**: the whole enabled fleet (`./scripts/scan_fleet.sh --list`) on `radio://0/80/2M` —
  workable because the log rates below are trimmed low. **When running two
  dongles** — as this rig historically did (cf1 on `radio://0/80/2M`, cf11 on
  `radio://1/90/1M`) — two rules, both proven the hard way here (see the
  comments in `config/crazyflies.yaml`):
  - **Channels ≥2 apart at 2M.** A 2M channel is ~2 MHz wide; adjacent
    channels make crazyflie-link-cpp refuse the pair outright:
    `Channels 80 and 81 are already served by Crazyradio 0`.
  - **Per-drone datarate must match the URI.** A `2M` URI on a drone whose
    radio runs at `1M` makes the server **hang forever at connect** — and the
    `/all/*` services (takeoff/land/arm) are never created. Verify with a scan
    on the drone's address (`./scripts/scan_fleet.sh`, or by hand `ros2 run crazyflie scan --address <address from the yaml>`).
- Keep total logging modest: `pose @ 10 Hz`, `status @ 1 Hz`, the
  `kalman_preflight` block `@ 5 Hz` is enough for monitoring. Add high-rate
  logging (50 Hz attitude/kalman
  blocks are commented out in the YAML) only when actively debugging, and remove
  it again for normal flight.
- **Symptoms of a saturated radio:** latency / receive-rate warnings from
  `crazyflie_server` (`min_unicast_receive_rate` in `config/server.yaml`),
  choppy position hold, dropped log packets, or the firmware supervisor
  triggering an emergency landing.

The config files live in `src/crazyswarm2/crazyflie/config/`. Edit them there, then
rebuild:

```bash
./scripts/build.sh crazyflie
```

## 4. Multicast vs unicast (primer)

- **Multicast** — Motive sends ONE stream to a multicast group address; every
  client that joins the group receives it. Needs healthy IGMP on the LAN (the
  switch/router must forward group joins) — and any process squatting on UDP
  1511 can silently starve the real client (see
  [TROUBLESHOOTING](TROUBLESHOOTING.md#mocap-pipeline)).
- **Unicast** — Motive sends a separate copy of the stream to each client. No
  IGMP dependency, at the cost of per-client bandwidth on the Motive PC.
- The number of **drones** is unaffected by the choice: all rigid bodies ride
  in every NatNet frame either way, and the drones themselves get their poses
  over the Crazyradio, not the network.
- The vendored `motion_capture_tracking` path used by `launch.py` (open
  `optitrack` parser, socket bound to `0.0.0.0:1511`) hears **Multicast** and
  Motive's **Broadcast Frame Data** (this rig: frames go to `255.255.255.255:1511`
  even though the Motive GUI says Multicast, because the profile flag
  `BroadcastInsteadOfMulticast` is set). It does **not** support Unicast. The
  `natnet_ros2` / closed-source NatNet SDK client binds the group address and is
  **deaf to broadcast** — that is why it sees nothing on this rig.

## 5. Networking: mocap over a router (lab setup)

![Flight laboratory network architecture: OptiTrack cameras → mocap router → Motive workstation, NatNet over wired LAN through the hangar switch to the lab D-Link router; ground control workstation joins via LAN, laptops/drones like the Starling Max 2 via 5 GHz Wi-Fi; Crazyflies are driven by Crazyradio from the workstation (software E-stop via Crazyswarm2), with an optional hardware E-stop remote](../Pics/Flight_lab_architecture.jpg)

*Flight lab network architecture — one-way arrows are data flow, plain lines are two-way connections. IPs are assigned per port, not per machine.*

The lab rig can also run with the mocap PC **wired to a lab router**, with the
ground-station laptop joining the **same subnet over 5 GHz Wi-Fi**. This network
carries only the **laptop ↔ Motive-PC link** — the drones ride the Crazyradio
and never join the subnet (Section 4):

- **Reserve static IPs via the router's DHCP** for the mocap PC and laptops so
  addresses stay stable across reboots.
- **Motive's streaming IP follows the active interface** — after switching the
  mocap PC (or your laptop) between wired and Wi-Fi, **restart Motive** so the
  streaming address updates; it does not reflect immediately.
- **Gigabit ports recommended** on the router/switch for the wired mocap PC.

**Concrete consequence:** switching your laptop between the router Wi-Fi and the
direct LAN **changes the Motive PC's address**, so `config/motion_capture.yaml`
must be updated to match:

- **`hostname:`** — leave it at `"auto"`: at launch, `launch.py` broadcasts a
  NatNet discovery ping on UDP 1510 and uses whichever Motive answers, logging
  `mocap: Motive at <ip> (from NatNet discovery)`. The lab Motive PC gets its
  address from DHCP and has moved several times (.100 → .124 → .152); every
  move used to leave `/poses` silent. If discovery fails, the launch **aborts
  with a message** instead of hanging. To pin an address without editing the
  file: `ros2 launch crazyflie launch.py mocap_hostname:=<ip>` or
  `export CRAZYSWARM_MOCAP_HOST=<ip>`. Discovery needs you on the Motive PC's
  LAN segment (broadcast does not cross routers) — `ip route get <ip>` must
  not say `via`.
- **`type: "optitrack"`** (the open parser) is **REQUIRED on Wi-Fi** — the
  closed-source parser wedges **permanently** on a Wi-Fi multicast stall and
  never recovers.

Then **restart Motive and relaunch** the stack. Symptom of forgetting:
`/poses` stays silent with **no error**.

ST Engineering staff: the lab-specific addresses and full router details
(credentials, IP plan, photos) are in the private repo:
[Mocap_QC_Ground_Control_Router_Information](https://github.com/AI-DA-STC/Mocap_QC_Ground_Control_Router_Information)
(link not confirmed as published; if it 404s, ask the lab admin for the router notes).
