# CrazySwarm2

Reproducible setup for an indoor **Crazyflie 2.1 swarm** flown with **OptiTrack**
motion-capture localization, built on [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2).

This is a **self-contained workspace**: the full source (our customized
`crazyswarm2` + `natnet_ros2`) is **vendored in [`src/`](src/)**, so a clone is a
byte-for-byte copy of the rig — no upstream fetching. One command installs the
dependencies and builds it on **Ubuntu 22.04 + ROS 2 Humble** or
**Ubuntu 24.04 + ROS 2 Jazzy**.

## Table of contents

1. [Overview and architecture](#1-overview-and-architecture)
2. [Hardware and supported platforms](#2-hardware-and-supported-platforms)
3. [Software setup](#3-software-setup)
   - [Step 1 — Install ROS 2](#step-1--install-ros-2-manual-setupsh-does-not-do-this)
   - [Step 2 — Clone and run the one-shot setup](#step-2--clone-and-run-the-one-shot-setup)
   - [Step 3 — Crazyradio USB permissions](#step-3--crazyradio-usb-permissions-manual-hardware-only)
   - [Step 4 — Simulator firmware bindings](#step-4--simulator-firmware-bindings-cffirmware-simulator-only)
   - [Step 5 — Activate and smoke-test](#step-5--activate-and-smoke-test)
4. [Configuring your fleet](#4-configuring-your-fleet)
5. [Preflight checks](#5-preflight-checks)
6. [Flying](#6-flying)
7. [Color LED control](#7-color-led-control)
8. [Mission console — optional GUI](#8-mission-console--optional-gui)
9. [Repo layout](#9-repo-layout)
10. [Documentation and troubleshooting](#10-documentation-and-troubleshooting)

## 1. Overview and architecture

![CrazySwarm2 with Motion Capture — system architecture: Motive streams NatNet multicast at 50 Hz to motion_capture_tracking, which publishes /poses to crazyflie_server; the server drives the Crazyflie fleet over Crazyradio #0 (radio://0/80/2M); user scripts (crazyflie_py) and the preflight GUI + RViz command and monitor it; alternative open-driver path: Motive → natnet_ros2 → /<body>/pose → pose_bridge.py → /poses](Pics/Crazyswarm_mocap_architecture.jpg)

> **Single-dongle fleet.** Five drones — `cf1`, `cf2`, `cf3`, `cf10` and `cf14` —
> are enabled, **all on ONE Crazyradio dongle** (`radio://0/80/2M`; addresses
> `0xE7E7E7E701/02/03/10/14`). `cf6` is commented out in the yaml —
> **dead on radio 2026-08-04** (silent on full channel/datarate sweeps at its
> own and the factory address; needs a physical check — do not re-enable until
> `scan --address 0xE7E7E7E706` answers). `cf5` and `cf11` are disabled spares.
> The earlier two-dongle cf1+cf11 setup is historical; its two hard-won rules
> still apply **when running two dongles** (rationale in the `crazyflies.yaml`
> comments and [CLAUDE.md → Gotchas](CLAUDE.md#gotchas-hard-won--dont-re-derive)):
> channels must be **≥2 apart at 2M** or crazyflie-link-cpp refuses the second
> radio, and the **URI datarate must match the drone** (cf11 talks 1M) — a
> mismatch makes the server hang forever with no `/all/*` services.

| Component | Source | Role |
|-----------|--------|------|
| `crazyswarm2` | github.com/IMRCLab/crazyswarm2 | Crazyflie swarm server, sim, examples, Python API |
| `natnet_ros2` | github.com/L2S-lab/natnet_ros2 | OptiTrack / NatNet driver |
| `motion_capture_tracking` | **vendored** in `src/motion_capture_tracking` (IMRCLab `ros2@64d3af2` + NatNet 4.2 patch — see [VENDORED.md](src/motion_capture_tracking/VENDORED.md); never the apt package) | Converts mocap data → `/poses` for the server |
| `pose_bridge.py` | this repo | Aggregates per-body poses into `NamedPoseArray` |
| `preflight_kalman_plotter.py` | this repo (`src/crazyswarm2/crazyflie/scripts/`) | Preflight GUI — per-drone go/no-go checks before flight |
| `src/` (vendored) | this repo | Customized crazyswarm2 + natnet_ros2 (configs, launch.py, scripts) |

## 2. Hardware and supported platforms

The rig this repo reproduces: **Crazyflie 2.1** drones, a **Crazyradio** dongle
(PA or 2.0) on the ROS 2 machine, and an **OptiTrack** camera system with
**Motive** on a Windows PC streaming NatNet multicast. Drones may optionally
carry the bottom-mounted **Color LED deck** (`bcColorLedBot`) — see
[Section 7](#7-color-led-control).

| Ubuntu | ROS 2 | Status |
|--------|-------|--------|
| 22.04 (Jammy) | Humble | supported |
| 24.04 (Noble) | Jazzy | supported |

The scripts auto-detect the pair from `/etc/os-release` (or an already-sourced
`$ROS_DISTRO`). Other combinations are rejected with a clear error.

## 3. Software setup

The full install lives here — there is no separate setup doc. `./scripts/setup.sh`
automates the workspace build, but **two things are deliberately left manual**
(marked below) because they are one-time, system-wide changes: installing ROS 2,
and Crazyradio USB permissions.

### Step 1 — Install ROS 2 *(manual; `setup.sh` does NOT do this)*

`setup.sh` requires ROS 2 to already be installed and will stop with an error if
it isn't. Install it once. The block below auto-detects the right distro
(Humble on 22.04, Jazzy on 24.04) — just copy-paste it whole:

```bash
# auto-detect the ROS 2 distro from your Ubuntu version
source /etc/os-release
case "$VERSION_ID" in
  22.04) export ROS_DISTRO=humble ;;
  24.04) export ROS_DISTRO=jazzy  ;;
  *) echo "Unsupported Ubuntu $VERSION_ID — need 22.04 or 24.04"; return 2>/dev/null || exit 2 ;;
esac
echo "Installing ROS 2 $ROS_DISTRO"

# enable the ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS 2 (desktop = includes RViz) and dev tools
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-desktop ros-dev-tools

# source it now and on every new shell
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
```

> Reference: official ROS 2 install guide —
> [Humble](https://docs.ros.org/en/humble/Installation.html) /
> [Jazzy](https://docs.ros.org/en/jazzy/Installation.html).

### Step 2 — Clone and run the one-shot setup

```bash
git clone https://github.com/AI-DA-STC/CrazySwarm2-with-Mocap.git ~/CrazySwarm2
cd ~/CrazySwarm2
./scripts/setup.sh
```

The source is already in `src/`, so `setup.sh` just:

1. `scripts/install_deps.sh` — apt deps + `rosdep install`. Also **removes** any apt `ros-<distro>-motion-capture-tracking`: the driver is vendored in `src/` because the apt 1.0.9 release hard-codes a foreign interface IP and never receives frames on lab laptops.
2. `scripts/build.sh` — `colcon build --symlink-install` at the repo root.

Re-run any step on its own later:

```bash
./scripts/install_deps.sh        # deps only
./scripts/build.sh               # rebuild everything
./scripts/build.sh crazyflie     # rebuild one package
LOW_MEM=1 ./scripts/build.sh     # serial build on low-RAM machines
```

`setup.sh` ends with a verification step and refuses to report success unless
`motion_capture_tracking` resolves to **this repo's** `install/` and the binary
is the vendored, fixed copy. If it prints an `ERROR`, fix that before going on.

#### Upgrading an existing clone (mocap driver vendored, September 2026)

If you cloned before the driver was vendored, do this **once** — otherwise your
laptop keeps running the apt driver and `/poses` stays silent even though the
Motive PC pings:

```bash
cd ~/CrazySwarm2
rm -rf src/motion_capture_tracking     # only if you cloned it there by hand (old fallback advice)
git pull
sudo apt remove -y ros-${ROS_DISTRO}-motion-capture-tracking ros-${ROS_DISTRO}-motion-capture-tracking-interfaces
rm -rf build install log               # the old build was linked against the apt interfaces package
./scripts/setup.sh                     # must end with "OK: motion_capture_tracking from .../install/..."
```

Then, in every shell you use, `source install/setup.bash` again. Quick check
at any time:

```bash
ros2 pkg prefix motion_capture_tracking   # must be inside this repo's install/, never /opt/ros
```

`launch.py` performs the same check and aborts with a clear message if the apt
copy would be used, so a wrong shell cannot silently fly without mocap.

> **⚠️ Install these too — `install_deps.sh` does not cover them yet.** The
> default launch and the Python API need a few extra packages:
>
> ```bash
> sudo apt install -y python3-tk python3-scipy ros-$ROS_DISTRO-joy
> ```
>
> Why each one: `python3-tk` → the **preflight GUI** window (started by
> default; without it the GUI dies or exits silently and no window appears),
> `ros-$ROS_DISTRO-joy` → the teleop `joy_node` (also default-on),
> `python3-scipy` → the `crazyflie_py` Python API (`hello_world` and every
> flight script). (`rowan` and a NumPy-2-compatible `matplotlib>=3.9` are
> already installed by `install_deps.sh`.)

### Step 3 — Crazyradio USB permissions *(manual; hardware only)*

Skip this if you only run the simulator. It lets your user talk to the Crazyradio
without `sudo` — `setup.sh` does NOT do this:

```bash
sudo groupadd plugdev 2>/dev/null; sudo usermod -aG plugdev $USER
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (PA) and Crazyradio 2.0
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie 2.x over USB
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
# log out / back in for the group change to take effect
```

Reference: [Bitcraze USB permissions guide](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/).

### Step 4 — Simulator firmware bindings (`cffirmware`) *(simulator only)*

The simulator backend (`backend:=sim`) imports the Crazyflie firmware Python
bindings (`cffirmware`) for software-in-the-loop control. These are **not** a pip
package and are **not** needed for hardware flight — only for the simulator. Build
them once (clones crazyflie-firmware outside the workspace, builds + installs the
bindings into your user site-packages):

```bash
./scripts/setup_sim_firmware.sh
source ~/.bashrc                       # the script adds the bindings to PYTHONPATH
cd ~ && python3 -c "import cffirmware; print('cffirmware OK')"   # verify (not from build/)
```

The script installs `git-lfs` (the firmware's CMSIS submodule needs it) and adds
the compiled bindings to `PYTHONPATH` via `~/.bashrc`, so open a new shell (or
`source ~/.bashrc`) before launching the simulator.

### Step 5 — Activate and smoke-test

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/CrazySwarm2/install/setup.bash
ros2 launch crazyflie launch.py backend:=sim   # needs Step 4; no hardware/mocap
```

> **Building by hand.** `setup.sh` already builds the workspace, but to (re)build
> manually — e.g. after editing source, or to see raw colcon output:
> ```bash
> cd ~/CrazySwarm2
> source /opt/ros/$ROS_DISTRO/setup.bash
> colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
> source install/setup.bash
> ```
>
> **Notes.** Do **not** `apt install ros-<distro>-motion-capture-tracking` — the
> driver is vendored in `src/motion_capture_tracking` (apt 1.0.9 is broken, see its
> [VENDORED.md](src/motion_capture_tracking/VENDORED.md)); if it is already installed,
> `sudo apt remove` it so it cannot shadow the workspace build, then verify
> `ros2 pkg prefix motion_capture_tracking` points into `install/`. On low-RAM
> machines use `LOW_MEM=1 ./scripts/build.sh`.

## 4. Configuring your fleet

The source (and its config) is vendored, so **edit files directly in `src/`** and
commit — a fresh clone then reproduces your exact rig. Key files:

- [`src/crazyswarm2/crazyflie/config/crazyflies.yaml`](src/crazyswarm2/crazyflie/config/crazyflies.yaml) — drone list, URIs, types,
  firmware logging, and each drone's `initial_position` (update it from `/poses`
  whenever a drone moves — procedure in
  [docs/MOCAP.md → Section 2b](docs/MOCAP.md#2b-setting-initial_position-from-poses)).
- [`src/crazyswarm2/crazyflie/config/motion_capture.yaml`](src/crazyswarm2/crazyflie/config/motion_capture.yaml) — Motive address, markers, QoS. `hostname: "auto"` (the default) makes `launch.py` find the Motive PC with a NatNet discovery ping, so DHCP drift on the lab LAN no longer matters; override any time with `mocap_hostname:=<ip>` or `export CRAZYSWARM_MOCAP_HOST=<ip>`: see [MOCAP Section 5](docs/MOCAP.md#5-networking-mocap-over-a-router-lab-setup).
- [`src/crazyswarm2/crazyflie/config/server.yaml`](src/crazyswarm2/crazyflie/config/server.yaml) — warning thresholds, sim backend/controller, `query_all_values_on_connect` (keep `True` — LED control needs the full param list at connect).
- [`src/crazyswarm2/crazyflie/config/teleop.yaml`](src/crazyswarm2/crazyflie/config/teleop.yaml) — gamepad mapping
  (see [docs/RUNNING.md → Section D](docs/RUNNING.md#d-manual--teleop-flight)).
- [`src/crazyswarm2/crazyflie/launch/launch.py`](src/crazyswarm2/crazyflie/launch/launch.py) — customized (adds the **foxglove_bridge**
  and **preflight GUI** nodes; `rviz` default `True`, `gui` default `False`).
- [`src/crazyswarm2/crazyflie/scripts/preflight_kalman_plotter.py`](src/crazyswarm2/crazyflie/scripts/preflight_kalman_plotter.py) — the preflight GUI
  (thresholds and takeoff/land setpoints are constants at the top; see
  [docs/RUNNING.md → Section C](docs/RUNNING.md#c-preflight-gui-preflight_kalman_plotterpy)).

[`pose_bridge.py`](pose_bridge.py) `DRONES` and `PUBLISH_HZ` must match `crazyflies.yaml` and the
Motive streaming rate — see [docs/MOCAP.md](docs/MOCAP.md).

> **Heads-up:** [`pose_bridge.py`](pose_bridge.py) currently lists `DRONES = ['cf1', 'cf2']`,
> which no longer matches the enabled fleet (`cf1`, `cf2`, `cf3`, `cf10`,
> `cf14`) — update it before using the alternative natnet_ros2 mocap path.

After editing anything in `src/`, rebuild: [`./scripts/build.sh`](scripts/build.sh) (or just the
changed package, e.g. `./scripts/build.sh crazyflie`).

## 5. Preflight checks

### Launch the server

First, in **Motive** set Data Streaming to **Multicast** at **50 Hz** — this must
match `src/crazyswarm2/crazyflie/config/motion_capture.yaml`. Activate the
workspace in **every** terminal you open (`source /opt/ros/$ROS_DISTRO/setup.bash
&& source ~/CrazySwarm2/install/setup.bash` — Step 5 above). Then:

```bash
# terminal 1 — Crazyflie server. Also starts mocap tracking, RViz, the
# preflight GUI and the Foxglove bridge (all on by default;
# rviz:=false / preflight:=False to disable)
ros2 launch crazyflie launch.py

# run the preflight checklist in the GUI (banner clear, mocap ~50 Hz,
# err.yaw ≈ 0°, battery green) — see docs/RUNNING.md → Preflight GUI
```

> **Rather not use the terminal?** `./console/run.sh` does this launch, the
> pre-launch radio scan and the checks below from a browser, showing each `ros2`
> command as it runs — see [section 8](#8-mission-console--optional-gui).

### What to expect after launch

`ros2 launch crazyflie launch.py` starts the server **and** three helper
surfaces (all on by default):

- **RViz** — you should see each drone's onboard EKF estimate (`cf1`) and its
  mocap frame (`cf1_mocap`) sitting **on top of each other**. Axes that are
  offset or rotated relative to each other mean a frame/orientation problem
  (see the misalignment example below).
- **Preflight GUI** (`crazyflie preflight`) — the per-drone go/no-go dashboard
  described next. One drone is shown at a time; **Prev/Next** (or ←/→) cycles
  through the enabled drones.
- **Foxglove bridge** — needs the package
  (`sudo apt install ros-$ROS_DISTRO-foxglove-bridge`) and is viewed from the
  Foxglove Studio app, not a window of its own.

If a drone is missing from the GUI it is `enabled: false` in `crazyflies.yaml`.
If the mocap-Hz trace never rises off zero, mocap isn't reaching the server —
see [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md#mocap-pipeline).

### The preflight GUI and checklist

The **preflight GUI** is the go/no-go check before *every* flight. It shows four
panels (kalman telemetry, pose/`stateEstimate`, connectivity, and
mocap-vs-onboard error) plus per-drone and all-drone **Takeoff / Land / Arm /
Disarm / E-STOP** buttons and **Record CSV**. **Green LED = drone connected and
ready for command. Red LED = drones are in flight / a script is controlling
them.** Mechanically (drones with the **Color LED deck**): the server sets
**green on connect**, any `crazyflie_py` script sets **red for its lifetime**
(green is restored on exit — normal return, exception or Ctrl-C), and the deck
goes **dark on clean shutdown**. So a lit LED is itself a "link up" preflight
signal, and red means keep hands clear. The deck can also be set to any color
manually — see [Section 7](#7-color-led-control). Full walkthrough:
[docs/RUNNING.md](docs/RUNNING.md#c-preflight-gui-preflight_kalman_plotterpy).

**Checklist — run through this before every test flight / debugging session:**

- [ ] **1. Check `crazyflies.yaml`** — in `crazyflie/config/crazyflies.yaml`, the drones you intend to fly are listed under `robots:` with `enabled: true` and shelf drones set to `enabled: false` — the server and preflight GUI only pick up enabled drones ([see example](#img-drone-yaml)). In the same file, check the `all: firmware_logging:` block if you need to track more data: right now only the `kalman_preflight` custom topic is active (plus the default `pose`/`status` topics); extra `custom_topics` can be un-commented/added, but each log block is limited to 26 bytes of variables ([see logging config](#img-logging-yaml)).
- [ ] **2. Drones on their yaml `initial_position`** — each drone physically sits at **its own** `initial_position` in `crazyflies.yaml` (update the yaml from `/poses` when you move a drone — procedure in [docs/MOCAP.md → Section 2b](docs/MOCAP.md#2b-setting-initial_position-from-poses)). Every pair of enabled drones must be **≥ 1 m apart**. Why this matters: the flight scripts compute **absolute** `goTo` targets from each drone's yaml `initial_position` (return-home = `initial_position` + height), so a drone placed at another drone's yaml position means crossing `goTo` paths — this caused a **real mid-air collision on 2026-08-04**. Never copy positions from `/cfX/pose` (circular — the estimate is seeded by the yaml); use `/poses` (mocap truth).
- [ ] **3. Check `motion_capture.yaml`** — in `crazyflie/config/motion_capture.yaml`, `hostname` is the IP of the PC running Motive **on the interface you're using** — the Wi-Fi and LAN addresses differ, see [MOCAP Section 5](docs/MOCAP.md#5-networking-mocap-over-a-router-lab-setup) (lab values live in the private router repo), `topics.frame_id` is `world`, `topics.tf.child_frame_id` is `{}_mocap`, and the Hz rate under `topics.poses.qos.deadline` **tallies with the camera frame rate set in OptiTrack's Motive software** — if Motive streams at a different Hz than the yaml expects, the deadline QoS flags the stream as unhealthy ([see mocap config](#img-mocap-yaml)).
- [ ] **4. Battery is not red** — header voltage shows green (or at worst orange); red < 3.7 V means charge or swap the pack first ([see battery example](#img-preflight-battery)).
- [ ] **5. Mocap and radio signal steady** — `mocap [Hz]` flat at ~50 Hz, RSSI steady, latency low and not spiking ([see healthy graph](#img-preflight-healthy)).
- [ ] **6. No error between mocap and drone orientation** — no red misalignment banner and dashed `err.yaw` near 0°. If misaligned, position the drone in the same orientation as the mocap rigid body — [how to align the drone axis to the mocap axis](#hangar-axes) — and recreate the rigid body in Motive ([see what a violation looks like](#img-preflight-misaligned)).
  > **Good practice:** each time a human enters the mocap zone, reset the rigid body in Motive — bumped markers or an occluded view can silently shift the body's orientation.
- [ ] **7. Kalman estimation converging near 0 at rest** — the kalman telemetry and error traces settle near zero while the drone sits still. If not, reset the drone by replugging its battery (or press **Reset Kalman (all)** / `r` in the GUI), then re-check ([see healthy example](#img-preflight-healthy)).

<a id="hangar-axes"></a>

**Axis alignment — drone ↔ mocap.** The GUI's `err.yaw` reads ~0° only when the
rigid body was created with the drone's body frame aligned to the mocap global
frame. How to do it:

1. **Find the drone's front.** On the Crazyflie the *Front direction* is the notch
   between motors **M1** and **M4** (the arm pair with the red/green LEDs; the PCB
   is marked `UP → FRONT`). Body frame: **X forward, Y left, Z up** — see the
   [Bitcraze coordinate-system reference](https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/).
2. **Find the hangar's +X.** Our mocap global frame is **Z-up** with **+X** running
   across the floor exactly as annotated in the photos below (origin at the cross
   on the carpet).
3. **Align and create.** Set the drone flat on the floor with its front pointing
   along the hangar's **+X**, then create (or reset) the rigid body in Motive
   while it sits like that. Done right, the dashed `err.yaw` trace sits at ~0°
   and the axes overlap in RViz.

*Crazyflie body frame — annotated on the real airframe (left: body axes
x̂_B/ŷ_B/ẑ_B, motor thrusts F₁–F₄ and spin directions, world frame in the corner)
and as a diagram with the roll/pitch/yaw rotations (right):*

<img src="Pics/crazyfly_axis_1.png" alt="Real Crazyflie with annotated body frame axes, motor thrust vectors F1-F4, rotor spin directions, and the world frame in the corner" width="49%"> <img src="Pics/crazyfly_axis_2.png" alt="Crazyflie body frame diagram: X forward (roll), Y left (pitch), Z up (yaw)" width="49%">

*Hangar mocap frame (+X across the floor, Z up; origin cross close-up on the right):*

<img src="Pics/mocap_axis_1.png" alt="Hangar capture volume with the mocap global axes annotated: x-axis red across the floor, y-axis green along the floor, z-axis blue pointing up" width="49%"> <img src="Pics/mocap_axis_2.png" alt="Top-down close-up of the floor origin cross: x-axis red, y-axis green, z-axis up out of the floor" width="49%">

### Emergency stop (E-STOP) demo

Every drone has an **E-STOP** button in the preflight GUI (and the `e` key) that
cuts motors instantly; **E-STOP (all)** kills the whole swarm at once. Know where
it is before you arm anything. Short demo:

![E-STOP demo — drone motors cut instantly from the preflight GUI](video/crazyfly_estop_draft1.gif)

▶️ [Full-quality video with audio](video/crazyfly_estop_draft1.mp4) (opens GitHub's player / downloads)

<!-- OPTIONAL upgrade to a real inline player with audio: drag
     video/crazyfly_estop_draft1.mp4 into the GitHub web editor for this README
     (pencil ✏️ → drag the file into the text area). GitHub uploads it and inserts
     a https://github.com/user-attachments/assets/<id> URL; paste that URL on its
     own line here and delete the GIF above if you prefer. -->

### Config files at a glance

The three config screenshots the checklist links to — what "correct" looks like
before you launch.

<a id="img-drone-yaml"></a>

*`crazyflies.yaml` — the `robots:` section. Each drone has its own block with a
per-drone `enabled: true/false` flag; only enabled drones are picked up by the
server and the preflight GUI:*

![crazyflies.yaml robots section — per-drone blocks with enabled true/false flags, URI and initial position](Pics/crazyswarm_drone_yaml.png)

<a id="img-logging-yaml"></a>

*`crazyflies.yaml` — the `all: firmware_logging:` block. `kalman_preflight` is the
active custom topic; the other `custom_topics` sit commented out, ready to enable
(each log block is capped at 26 bytes of variables):*

![crazyflies.yaml firmware_logging block — kalman_preflight custom topic enabled, other custom topics commented out ready to enable](Pics/crazyswarm_logs_yaml.png)

<a id="img-mocap-yaml"></a>

*`motion_capture.yaml` — `hostname` (Motive PC IP), `topics.frame_id` (`world`),
`topics.tf.child_frame_id` (`{}_mocap`) and the poses QoS `deadline` Hz, which
must match the camera frame rate set in Motive. The `hostname` and `type` values
in the screenshot are illustrative — use your rig's current values per
[MOCAP Section 5](docs/MOCAP.md#5-networking-mocap-over-a-router-lab-setup):*

![motion_capture.yaml — hostname set to the Motive PC IP, frame_id world, child_frame_id {}_mocap, and the poses QoS deadline rate that must match Motive's camera frame rate](Pics/motion_capture_yaml.png)

### Reading the GUI

Clear the drone against these before arming:

| Check | Where | Healthy | Go / no-go |
|-------|-------|---------|-----------|
| **Battery** | header voltage | **green > 3.8 V** | **orange 3.7–3.8 V** = fly but land soon; **red < 3.7 V** = **NO-FLY** (charge/swap). `(charging)` = NO-FLY until unplugged. |
| **Mocap rate** | connectivity → `mocap [Hz]` | flat **~50 Hz** | sagging or decaying toward 0 = mocap dying → **NO-FLY** |
| **Radio** | connectivity → `rssi` / `latency` | rssi steady, latency low & flat | erratic / climbing latency = saturated radio (lower logging/mocap rate) |
| **Orientation** | error panel → dashed `err.yaw` + red banner | **\|err.yaw\| ≤ 5°** | 5–15° **fix first**; **> 20° NO-FLY**; the red *"MOCAP ORIENTATION MISALIGNED"* banner fires above 10° |
| **Estimator fusion** | error panel → `err.x/y/z/norm` | small & steady (~mm) | diverging = Kalman not fusing mocap → **NO-FLY** |

> **Don't be reassured by tiny position error.** `locSrv.extPosStdDev` force-fuses
> mocap position, so `err.x/y/z` stays ~1 mm *even when the rigid body is defined
> rotated* — that offset is invisible at rest but becomes a **fly-away** in the
> air. The orientation error (`err.yaw`) and the banner are what catch it. Press
> **`r`** to reset the Kalman filter (all drones); **`e`** is E-STOP.

<a id="img-preflight-healthy"></a>

**Healthy preflight (Flow deck fitted).** All four panels steady, `mocap ~50 Hz`,
`err.yaw` ≈ a couple of degrees. Battery here is 3.71 V (orange) — flyable, but
plan to land and swap soon.

![Healthy preflight with a Flow deck fitted — steady traces, mocap 50 Hz, small err.yaw](Pics/preflight-healthy-flowdeck.png)

**Healthy preflight (no Flow deck).** With no Flow deck the flow/range channels
(`motion.deltaX/Y`, `range.zrange`, `stateEstimateZ.vx/vy`) sit flat or noisy —
**this is expected, not a fault.** Judge such a drone on mocap, battery and yaw.

![Healthy preflight without a Flow deck — flow/range channels idle, which is normal](Pics/preflight-healthy-no-flowdeck.png)

<a id="img-preflight-battery"></a>

**Warning-band battery.** 3.72 V shows orange: still flyable, but you're near the
warning threshold — do a short flight and recharge.

![Preflight GUI showing an orange warning-band battery at 3.72 V](Pics/preflight-battery-warning.png)

<a id="img-preflight-misaligned"></a>

**NO-FLY — orientation misaligned + low battery.** The red banner reads
*"MOCAP ORIENTATION MISALIGNED: cf2 (yaw 18°)"*, `err.yaw` (dashed) climbs toward
15°, and RViz (right) shows the `cf2_mocap` axes visibly rotated. Battery is
3.64 V (red). Do **not** fly: recreate the Motive rigid body with the drone's
forward axis on global **+X** ([docs/MOCAP.md](docs/MOCAP.md#2-defining-rigid-bodies))
and charge the pack.

![NO-FLY example — red orientation-misalignment banner, err.yaw ~15°, rotated RViz axes, red 3.64 V battery](Pics/preflight-nofly-misaligned.png)

### What a flight looks like

During takeoff/hover/land the kalman, pose and error panels show real motion —
transient swings and error spikes while the drone moves — then settle back when
it lands. **`mocap [Hz]` should stay pinned at ~50 Hz the whole time**; a drop
mid-flight is the mocap failure that triggers an emergency land. Note the battery
sags under load (3.68 V, red, after the flight below):

![Preflight GUI during/after a flight with a Flow deck — motion in the telemetry panels, mocap held at 50 Hz, battery drained to 3.68 V](Pics/preflight-during-flight.png)

## 6. Flying

### First flight — hello_world

With the server launched and the checklist green (Section 5):

```bash
# terminal 2 — takeoff, hover, land
ros2 run crazyflie_examples hello_world
```

Hover/landing height and durations are set in `hello_world.py` — see
[docs/RUNNING.md](docs/RUNNING.md#adjusting-the-flight-hello_worldpy).

### Beyond hello_world — flight tools

Quick reference only; launch flows live in [docs/RUNNING.md](docs/RUNNING.md),
and each tool's header docstring is its full manual:

- **Manual flight, Xbox controller** — `ros2 run crazyflie_examples teleop_xbox`
  (start the server with `teleop:=False` so only this script owns the pad).
  Sticks steer a **position setpoint** with a **geofence**: auto-land on
  leaving the fence radius or on stale mocap, height clamped, refuses takeoff
  without a live mocap pose; **B = emergency** (motors cut, drone drops).
  Full flow: [RUNNING Section D](docs/RUNNING.md#teleop_xbox-geofenced-position-teleop);
  generic gamepad teleop: [RUNNING Section D](docs/RUNNING.md#d-manual--teleop-flight).
- **Multi-drone trajectory demos** — `ros2 run crazyflie_examples
  multi_trajectory` (whole fleet flies traj1 in formation, returns home, slow
  landing, ~50 s) and `ros2 run crazyflie_examples multi_trajectory_formation`
  (no traj1 — instead a formation dance: waypoint tour (3 waypoints + return,
  rigid group moves) → pentagon gather → one smooth 360° spin
  (10 s) → triangle+tail morph → whole-swarm orbit of the room center
  (1.2 m radius, 12 s, ~0.63 m/s) → back to the pentagon → home, ≈ 58.0 s
  airborne — needs a **~2.24 m clear radius** around the room center).
  In sim add `--ros-args -p use_sim_time:=true`; on hardware run without it.
  Full flow: [RUNNING Section B](docs/RUNNING.md#multi-drone-trajectory-demos).

  ![Formation demo — five drones fly the waypoint tour, pentagon gather, 360° spin, triangle+tail morph, and room-center orbit](video/formation_demo_1.gif)

  *Formation demo flight footage.*

  ![Formation demo — second recording of the multi_trajectory_formation flight](video/formation_demo_2.gif)

  *Formation demo flight footage, second angle (first 60 s).*
- **Prop-spin ground test** — `ros2 run crazyflie_examples arming` arms and
  spins all four props at low PWM (~15%, far below hover) for 10 s, then stops
  and disarms — motors always stopped even on Ctrl-C. **Ground test only**:
  drone on the floor, fingers clear.
  Full flow: [RUNNING Section E](docs/RUNNING.md#prop-spin-ground-test-arming-example).
- **LED tools** — `./scripts/led.sh <color>` and
  `ros2 run crazyflie_examples color_led <color>` set the Color LED deck live
  while the server runs (both also have an interactive number-key mode);
  `scripts/color_led_cflib.py` talks directly over cflib — **stop the server
  first**, they cannot share the Crazyradio.
  Full flow: [RUNNING Section E](docs/RUNNING.md#color-led-deck--status-convention-and-manual-control).
- **Runtime firmware params** — `ros2 param set /crazyflie_server
  cf1.params.<group>.<name> <value>` now reaches the drone immediately: the
  vendored server pushes params from an on-set callback (upstream's
  `/parameter_events` path silently never fired on this rig — see
  [CLAUDE.md → Gotchas](CLAUDE.md#gotchas-hard-won--dont-re-derive)).
  Full flow: [RUNNING Section E](docs/RUNNING.md#runtime-firmware-parameters).

## 7. Color LED control

Drones carrying the bottom-mounted **Color LED deck** (`bcColorLedBot`) double
as swarm status lights. Two things happen automatically:

- **Green on connect** — the server lights the deck full green when a drone
  connects (and it goes dark on clean shutdown).
- **Red while a script runs** — any `crazyflie_py` script (`Crazyswarm()`)
  turns the deck **red** for its lifetime and restores **green** on exit —
  normal exit, exception, or Ctrl-C alike.

Colors can also be set manually while the server is running, by name or by
number key (same mapping in every tool):

| Key | `0` | `1` | `2` | `3` | `4` | `5` | `9` |
|-------|-----|-----|-----|-----|-----|-----|-----|
| Color | off | green | red | yellow | blue | purple | white |

```bash
# easiest — bash wrapper over `ros2 param set` (most reliable path)
./scripts/led.sh yellow      # one-shot, by name (case-insensitive)
./scripts/led.sh 3           # one-shot, by number key (= yellow)
./scripts/led.sh             # interactive: press 0-5/9 to switch, q quits

# same thing as a ROS 2 node (workspace built + sourced)
ros2 run crazyflie_examples color_led yellow
ros2 run crazyflie_examples color_led        # interactive

# standalone cflib demo — STOP the server first (they can't share the radio)
python3 scripts/color_led_cflib.py           # fixed color sequence; URI edited in-file
```

All paths set the firmware parameter `colorLedBot.wrgb8888` (`0xWWRRGGBB`, with
a dedicated white channel). **Hardware only** — no effect with `backend:=sim`.
Full detail (raw `ros2 param set` form, decimal color values, prerequisites):
[docs/RUNNING.md Section G](docs/RUNNING.md#g-color-led-control-color-led-deck).

## 8. Mission console — optional GUI

Everything in sections 4–7 can also be driven from a browser:

```bash
./console/run.sh          # → http://localhost:8077
```

It sources ROS and this workspace for you, then runs the **same `ros2` commands**
as child processes — and shows each one before it runs, so you can copy it into a
terminal instead of memorising it. Five tabs:

- **System health** — a diagram of the real data path (workspace → config →
  radio + Motive → `/poses` → `crazyflie_server` → each drone). Each box is its
  own probe; click one for what was measured, the command behind it and the fix.
  It names the silent failures this rig actually hits — a leftover socket on UDP
  1511, the apt mocap driver shadowing the vendored one, the server blocked
  forever mid-connect with no `/all/*` services, a URI datarate mismatch, a conda
  python — instead of leaving them for you to find in the launch log.
- **Control** — the launch (every argument), the preflight scans and checks, the
  flight examples, `/all/*` service calls, firmware params, LED, and recovery
  actions. Anything that moves a drone confirms first; E-STOP is always in the
  header.
- **Config** — edit `crazyflies.yaml` as a table (enable/disable, URI,
  `initial_position`, type, add/remove a drone) or any config file as raw text.
  Writes are parse-checked, diffed, backed up, and **keep the comments**.
- **Processes** — live output of everything it started, with stop/kill.
- **Command log** — every command of the session, downloadable as a runnable
  shell script.

It is a **module, not a fork**: it lives entirely in [`console/`](console/),
nothing in `src/` references it, and it needs no build — `rm -rf console/` removes
it and the workspace still works exactly as this README describes. Details,
including the "no authentication" caveat before you bind it to the lab network:
[console/README.md](console/README.md).

## 9. Repo layout

```
CrazySwarm2/
├── src/                  # VENDORED source — crazyswarm2 + natnet_ros2 (committed)
├── scripts/              # setup.sh, install_deps.sh, build.sh, setup_sim_firmware.sh,
│                         #   led.sh (LED via ros2 param set), color_led_cflib.py (direct cflib)
├── pose_bridge.py        # natnet → /poses bridge (50 Hz)
├── console/              # OPTIONAL mission-console GUI (section 8) — delete it and
│                         #   everything above still works; nothing depends on it
├── docs/                 # RUNNING, MOCAP, TROUBLESHOOTING
├── CLAUDE.md
└── build/  install/  log/   # generated, git-ignored
```

### Provenance

`src/` was vendored from these upstreams (with local customizations):

- `crazyswarm2` — github.com/IMRCLab/crazyswarm2 (`ae23edc`)
- `natnet_ros2` — github.com/L2S-lab/natnet_ros2 (`883b095`)

To pull upstream changes, diff against those and merge manually, or re-vendor.

## 10. Documentation and troubleshooting

- [docs/RUNNING.md](docs/RUNNING.md) — sim, hardware, mocap launch flows; the preflight GUI; custom logging; Color LED control
- [docs/MOCAP.md](docs/MOCAP.md) — OptiTrack calibration, rigid bodies, 240→50 Hz tuning; see also [Networking: mocap over a router (lab setup)](docs/MOCAP.md#5-networking-mocap-over-a-router-lab-setup)
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — common failures
