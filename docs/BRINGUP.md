# Bring-up runbook: Starling 2 Max / VOXL 2

Ordered, bench-level bring-up procedure for a fresh or newly-flashed Starling
2 Max. **Follow the stages in order and do not skip a checkpoint** -- several
later stages (voxl-mapper, the follower stack) silently misbehave rather
than cleanly failing if you start them against an unhealthy dependency
(most importantly qVIO -- see Stage 3).

Conventions used below:
- `$` prefixes a command to run in an SSH shell on the VOXL 2 (`adb shell`
  or `ssh root@<voxl-ip>` depending on your connection method).
- Each stage ends with a **Checkpoint** -- the concrete thing to confirm
  before moving to the next stage.
- A full troubleshooting table is at the end of this document.

---

## Stage 0 -- Before you start

- [ ] VOXL 2 / Starling 2 Max on bench power or a charged, connected battery.
- [ ] Host laptop on the same network as the VOXL 2 (USB tethered, or Wi-Fi
      hotspot mode -- confirm with `ping <voxl-ip>` from the host).
- [ ] `adb` or SSH access confirmed: `adb shell echo ok` (or
      `ssh root@<voxl-ip> echo ok`) returns `ok`.
- [ ] Propellers **removed** for every stage in this document except the
      final flight check. Nothing here requires props on.

---

## Stage 1 -- Flash and verify the VOXL SDK

1. Confirm the currently installed SDK image and package versions:
   ```
   $ voxl-version
   ```
   Expected output: a block listing `Image:` (the flashed system-image
   version), followed by `Kernel:`, `VOXL-SDK:`, and per-package versions.
   Record the `VOXL-SDK:` version -- you will want it in any bug report.

2. If this is a fresh unit, or you are intentionally re-flashing (recovering
   from a bad state, or moving to a new SDK release), flash per ModalAI's
   fastboot procedure for the Starling 2 Max (`voxl-flash` from the
   ModalAI-provided flashing package on the host, with the VOXL 2 in
   fastboot mode). This project assumes the SDK is **already flashed** by
   the time this runbook starts; the flashing procedure itself is
   hardware/host-OS specific and is documented in ModalAI's official
   flashing guide for your exact carrier board -- follow that guide, then
   resume here.

3. After flashing (or on first bring-up of an already-flashed unit), confirm
   all core VOXL services are present and enabled:
   ```
   $ voxl-inspect-services
   ```
   Expected output: a table of systemd services (`voxl-camera-server`,
   `voxl-qvio-server`, `voxl-vision-hub`, `voxl-portal`, ...) each showing
   `enabled` and, for the ones already running by default, `running` with a
   nonzero, sane CPU% (not pegged at or near 100%).

4. Confirm basic camera capture works before anything depends on it:
   ```
   $ voxl-inspect-cam
   ```
   Expected output: a live-updating per-camera frame rate close to the
   configured rate for each enabled camera in the C29 (stereo + ToF)
   config. `Ctrl-C` to exit.

**Checkpoint:** `voxl-version` reports a version, `voxl-inspect-services`
shows the core services `enabled`, and `voxl-inspect-cam` shows live frame
rates for every camera in the C29 config. Do not proceed on a unit where any
camera shows 0 fps -- fix that first (reseat the cable, re-run
`voxl-configure-cameras`).

---

## Stage 2 -- Install ROS 2 Foxy

The Starling ships PX4 running on the sensors DSP and the VOXL-side stack
talking MPA (Modal Pipe Architecture), not ROS 2, out of the box. ROS 2
Foxy and the MPA<->ROS 2 bridge are added on top.

1. Install ROS 2 Foxy from the on-device package cache:
   ```
   $ apt-get install voxl-ros2-foxy
   ```
   **This does not need internet access.** The `.deb` packages for
   `voxl-ros2-foxy` and its ROS 2 dependencies are already staged on the
   VOXL 2's filesystem (part of the shipped SDK image / offline apt cache);
   `apt-get` is resolving and installing from that local cache, not
   downloading from the network. If you see DNS or "could not resolve
   archive.ubuntu.com" errors, that is `apt-get` incorrectly trying to hit
   a remote mirror for something *not* in the local cache -- see the
   troubleshooting table.

2. Source the ROS 2 environment for the current shell, and add it to
   `~/.bashrc` on the VOXL 2 so every future shell has it automatically:
   ```
   $ source /opt/ros/foxy/setup.bash
   $ echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
   ```

3. Confirm the install:
   ```
   $ ros2 doctor --report | head -20
   ```
   Expected output: no fatal errors; a report block showing ROS
   `distro: foxy` and the RMW implementation in use.

**Checkpoint:** `ros2 doctor --report` runs without a "ROS 2 not sourced"
style error, in a **fresh** shell (i.e. after the `.bashrc` change, open a
new shell and re-run, don't just trust the shell you sourced it in
manually).

---

## Stage 3 -- Start voxl-mpa-to-ros2 and voxl-microdds-agent

These two services are what let ROS 2 nodes (the follower stack in
`ros2_ws/`) see VOXL's MPA data (cameras, VIO, ToF, ...) and talk to PX4,
respectively.

1. Configure and enable the MPA-to-ROS2 bridge:
   ```
   $ voxl-configure-mpa-to-ros2
   ```
   This writes the bridge's topic-mapping config and enables its systemd
   service. Note: ModalAI's own setup notes call for internet access
   specifically for this configure step on some SDK versions (it may `pip
   install` a small number of Python dependencies) -- this is a different
   requirement from Stage 2's apt install, which is fully offline. If your
   bench has no internet and this step fails on a pip install, do this step
   once on a networked bench, or vendor the wheels ahead of time.

2. Install and configure the MicroDDS agent (the ROS 2 <-> PX4 uXRCE-DDS
   bridge):
   ```
   $ apt-get install voxl-microdds-agent
   $ voxl-configure-microdds
   ```

3. Confirm both services are running:
   ```
   $ systemctl status voxl-mpa-to-ros2
   $ systemctl status voxl-microdds-agent
   $ voxl-inspect-services
   ```
   Expected: both show `active (running)` in `systemctl status`, and both
   appear `running` in the `voxl-inspect-services` table.

4. Confirm ROS 2 actually sees data flowing, from a shell with ROS 2
   sourced (Stage 2):
   ```
   $ ros2 topic list
   $ ros2 topic hz /fmu/out/vehicle_odometry
   ```
   Expected: `ros2 topic list` includes PX4 `/fmu/...` topics (from
   MicroDDS) and MPA-bridged topics (camera/VIO topics, exact names per
   your `voxl-configure-mpa-to-ros2` mapping); `ros2 topic hz` reports a
   steady, nonzero rate once PX4 is powered and the DDS link is up.

**Checkpoint:** `ros2 topic list` shows both PX4 (`/fmu/...`) and MPA-bridged
topics, and `ros2 topic hz` on at least one PX4 topic reports a steady rate.

---

## Stage 4 -- Verify qVIO is healthy BEFORE anything else

**Do this before starting voxl-mapper, voxl-vision-hub's offboard mode, or
any follower code.** voxl-mapper hard-depends on VIO, and
`voxl-vision-hub.conf`'s `vio_warmup_s: 3.0` (see
`config/voxl-vision-hub.conf.md`) means the whole stack treats VIO as
"not good yet" for the first 3 continuous seconds of lock even once it
looks fine on screen -- starting things early just means they start against
a still-warming-up estimate.

1. With the vehicle stationary on a flat surface, in view of textured
   surroundings (qVIO needs visual features -- do not point it at a blank
   wall or the ceiling only), run:
   ```
   $ voxl-inspect-qvio
   ```
   Expected healthy output, updating live, looks like:
   ```
   T_imu_wrt_vio (m)   |Roll Pitch Yaw (deg)| state| error_codes
   -0.02    0.01    0.00|   0.3   -0.1    0.4| OKAY |
   ```
   The `state` column is the field to watch: it must read `OKAY`, not
   `INITIALIZING`, `ERROR`, or blank.

2. Pick the vehicle up and move it by hand through a few tens of
   centimetres in each axis (translate in X/Y/Z, small rotations), watching
   the `T_imu_wrt_vio` and `Roll Pitch Yaw` columns track the motion
   smoothly and return to near-zero drift when set back down. Position
   values should move by roughly the distance you moved it (a rough
   tape-measure check is enough -- you are checking for gross scale/sign
   errors, not calibration-grade accuracy).

3. Let it sit still and confirm `state` holds `OKAY` continuously for **at
   least 5 seconds** (comfortably past the `vio_warmup_s: 3.0` gate) before
   moving on.

**Checkpoint:** `voxl-inspect-qvio` shows `state: OKAY` continuously for
5+ seconds while stationary, and tracks hand-carried motion sensibly. If you
cannot get a stable `OKAY`, stop here -- fix VIO first (better lighting,
more visual texture in view, check camera calibration) before touching
voxl-mapper. Everything past this point assumes qVIO is trustworthy.

---

## Stage 5 -- Stand up voxl-mapper standalone, fly a planned path via voxl-portal

With qVIO confirmed healthy (Stage 4):

1. Confirm `config/voxl-mapper.conf` (this repo) matches, or has been
   copied to, `/etc/modalai/voxl-mapper.conf` on the vehicle, and likewise
   for `config/voxl-vision-hub.conf`. See the sibling `.md` files next to
   each for what every key does.

2. Restart both services so they pick up the config:
   ```
   $ systemctl restart voxl-vision-hub
   $ systemctl restart voxl-mapper
   $ voxl-inspect-services
   ```
   Expected: both show `running` with sane (not pegged) CPU%.

3. Open `voxl-portal` in a browser on the host laptop (default
   `http://<voxl-ip>:7000`, confirm the exact port with
   `voxl-configure-portal --help` or your SDK's release notes if it
   differs). Confirm:
   - The 3D map view populates with an occupancy/mesh view of the room as
     you pan the camera view around it (proof `voxl-mapper` is consuming
     depth data and building a map, not just idling).
   - The vehicle's own pose marker in the map tracks smoothly, consistent
     with Stage 4's qVIO check.

4. **With props still off**, use voxl-portal's path-planning UI to click a
   goal point a metre or two from the vehicle's current position inside the
   mapped room, and request a plan. Confirm:
   - A path appears in the 3D view routed around any mapped obstacle
     between the vehicle and the goal (not a straight line through a wall
     or furniture).
   - `ros2 topic echo /fmu/in/trajectory_setpoint --once` (or the
     equivalent plan_msgs / mapper-output topic name your bridge maps it
     to) shows a nonzero, moving setpoint once the plan is accepted --
     confirms voxl-mapper -> voxl-vision-hub -> PX4 trajectory relay is
     actually wired end-to-end, independent of whether the vehicle is
     armed.

5. Only once the above is confirmed with props off, do a props-on flight
   test of a short planned path in a clear area, with a safety pilot ready
   on the RC per `docs/SAFETY.md`, and confirm the vehicle physically
   follows the path shown in voxl-portal.

**Checkpoint:** voxl-portal shows a populated map, a routed (not
straight-line) plan around an obstacle, and a live trajectory setpoint on
the ROS 2 side once a plan is accepted. Do not proceed to follower-stack
integration testing until this stage is clean.

---

## Stage 6 -- Verify voxl-tflite-server

### Hard warning: TFLite delegates fail SOFT, not hard

**If the delegate `voxl-tflite-server` is configured to use (GPU, NNAPI,
Hexagon/DSP) cannot actually bind at startup, the server does not error
out or refuse to start -- it silently falls back to running the model on
the CPU.** A silently-CPU-bound detector still produces bounding boxes, so
a naive smoke test ("is it publishing detections?") will pass even in this
failure mode. What actually happens is: inference latency multiplies,
`voxl-tflite-server`'s CPU usage pegs a core (or more), and everything else
sharing that CPU budget (qVIO, voxl-mapper, the follower's control loop)
gets starved of cycles -- a failure that shows up as *sluggish, jittery
flight behaviour* far from the tflite server itself, which makes it a nasty
one to root-cause after the fact if you didn't rule it out at bring-up.

**Bring-up MUST explicitly assert the requested delegate actually bound.**
Do not accept "it's producing detections" as sufficient.

1. Stop the service and run it in the foreground so you can read its
   startup log directly:
   ```
   $ systemctl stop voxl-tflite-server
   $ voxl-tflite-server
   ```
   Read the startup block for an explicit delegate-bind line (naming the
   delegate you configured -- GPU/NNAPI/CPU -- and confirming it applied),
   versus a fallback/error line saying delegate creation failed and it is
   continuing on CPU. Do not just check that frames start processing.

2. Cross-check with the actual per-inference latency, since the log
   message alone is worth confirming against a number:
   ```
   $ voxl-inspect-services -v
   ```
   or watch the tflite-server's own reported per-frame inference time in
   its log/telemetry output. **The shipped YOLOv8/YOLOv11 detection models
   are known to fail delegation on this platform** (delegate init errors on
   the custom detection-head ops) and silently run on CPU regardless of
   configuration -- do not trust a "successful" run of a YOLOv8/v11 model
   as proof the delegate path works. **MobileNet-SSD is the model known to
   delegate cleanly**, at roughly **7.72 ms** per inference when the
   delegate has actually bound. Use MobileNet-SSD as your bring-up
   reference model specifically *because* it is the one model where a
   clean delegate bind is achievable and verifiable: run it, confirm the
   startup log's delegate-bind line, and confirm the per-inference time is
   in that ballpark (tens of ms or more, not ~8 ms, means you are silently
   on CPU regardless of what the log said).

3. Once the delegate bind is confirmed with MobileNet-SSD, re-start the
   service normally for whatever model the follower stack actually uses in
   flight:
   ```
   $ systemctl start voxl-tflite-server
   ```
   and repeat the log check for that model. If that model is a YOLOv8/v11
   variant, budget for it running on CPU (see the troubleshooting table)
   rather than assuming delegation will save you at flight time.

**Checkpoint:** the tflite-server startup log shows an explicit successful
delegate-bind message (not a fallback message) for the model you intend to
fly with, corroborated by an inference-time number consistent with that
delegate (≈7.72 ms for MobileNet-SSD on a working delegate; much higher on
CPU). If the flight model is YOLOv8/v11 and it cannot be made to delegate,
document that explicitly as a known CPU-bound path in your test log, not a
silent assumption.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `apt-get install voxl-ros2-foxy` tries to hit the network / DNS failure | `apt-get` is looking for a package (or a dependency) not present in the local offline cache | Confirm you're installing the exact package name shipped for your SDK version; check `apt-cache policy voxl-ros2-foxy` for available versions locally before assuming you need network access |
| `voxl-configure-mpa-to-ros2` fails on a pip install step | This specific configure step (unlike the base package install) needs internet for a couple of Python deps on some SDK versions | Run this one step once with the bench networked, or pre-vendor the wheels; it is not required for every stage in this document, only this one |
| `ros2 topic list` empty / no `/fmu/...` topics | `voxl-microdds-agent` not running, or PX4 not powered/booted yet | `systemctl status voxl-microdds-agent`; confirm PX4 boot with `voxl-inspect-services`; power-cycle if the FMU side never came up |
| `voxl-inspect-qvio` stuck at `INITIALIZING` | Not enough visual texture in view, camera not calibrated, or camera server not running | Point at a textured scene, not a blank wall/ceiling; confirm `voxl-inspect-cam` shows live frames first; re-run camera calibration if this persists |
| `voxl-inspect-qvio` shows `OKAY` but drifts badly when moved | Bad extrinsics/calibration, or vibration coupling into the IMU | Re-run camera-IMU calibration; check mounting for a loose accessory rattling near the IMU |
| voxl-mapper running but voxl-portal map view stays empty | Depth pipe(s) misconfigured in `voxl-mapper.conf` (wrong `tof_pipe`/`depth_pipe_*` name), or qVIO not actually `OKAY` yet when voxl-mapper started | `voxl-list-pipes` to confirm the real pipe names match `config/voxl-mapper.conf`; restart voxl-mapper only after Stage 4's checkpoint is clean |
| Planned path in voxl-portal is a straight line through an obstacle | ESDF hasn't built up yet for that region (too soon after starting, or that area never observed by a sensor) | Pan the camera to observe the region first, wait a few seconds for the ESDF to populate, replan |
| `voxl-tflite-server` produces detections but flight is sluggish elsewhere | **Silent CPU fallback** -- delegate never bound, CPU pegged, starving other services (see Stage 6) | Re-run Stage 6's startup-log check; do not trust "it's producing boxes" as evidence the delegate bound |
| YOLOv8/v11 model won't delegate no matter what's tried | Documented platform limitation -- these models are known to fail delegation on this TFLite/hardware combination | Use MobileNet-SSD for anything that needs a verified low-latency delegate path; if the flight model must be YOLOv8/v11, plan for CPU-bound inference time and re-budget the perception loop rate accordingly |
| MobileNet-SSD inference time reads tens of ms, not ~7.72 ms | Delegate silently fell back to CPU despite configuration | Recheck `voxl-tflite-server`'s config `delegate` key and the startup log; confirm no other process is holding the GPU/NNAPI resource |
