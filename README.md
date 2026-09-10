# swarm-shows — working directory

Development copy of the `crazyflie_shows` ROS 2 package. Choreography built on
`crazyflie_py`, kept outside the main workspace so it can be edited freely and
dropped in when it is ready to fly.

**Three documents, in reading order:**

| file | what it covers |
|---|---|
| `crazyflie_shows/HANDOVER.md` | the **rig** — mocap, radios, the traps. Read §4a and §6 first |
| `crazyflie_shows/SHOW_GUIDE.md` | the **show** — what it does, what to fill in, how to deploy and fly |
| this file | the working directory and the drop-in |

## What is in here

A ~62 s, five-drone show that opens with the formation barely moving and ends
with the fastest, widest figure in the piece: breathe → slow carousel → wave →
reversed carousel → counterflow → fast carousel → bloom finale. Full
description and deployment procedure in `SHOW_GUIDE.md`.

## Drop-in

The package is relocatable: no absolute paths, every resource resolved through
`get_package_share_directory()`, all dependencies declared in `package.xml`.

```bash
cp -r crazyflie_shows ~/CrazySwarm2-with-Mocap/src/crazyswarm2/
cd ~/CrazySwarm2-with-Mocap
./scripts/build.sh crazyflie_shows
source install/setup.bash
ros2 pkg prefix crazyflie          # MUST be this workspace, never /opt/ros/humble
```

Because the workspace is built with `--symlink-install`, once the package has
been built **editing a `.py` body needs no rebuild**. Rebuild only when adding
a new file, a new `setup.cfg` entry point, or a new data file. Editing
`config/*.yaml` needs no rebuild but does need a launch restart.

## Run

```bash
# verify the show on the ground - no radio, no mocap, no drones, no ROS graph
ros2 run crazyflie_shows plan_show
ros2 run crazyflie_shows plan_show --plot        # writes show_plan.png

# sim. The sim clock is ~4x slower than wall time, so use_sim_time is required.
ros2 launch crazyflie_shows show_launch.py backend:=sim
ros2 run crazyflie_shows swarm_show --ros-args -p use_sim_time:=true

# hardware: stack up, preflight GUI checked, THEN the show
ros2 launch crazyflie_shows show_launch.py
ros2 run crazyflie_shows swarm_show
```

`plan_show` builds the *same* plan object `swarm_show` flies and checks every
budget against it, so it is the cheapest way to find out that a choreography
edit is unsafe. It refuses rather than warns.

`plan_show` also runs with no ROS at all, straight from the source tree:

```bash
PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \
    python3 -m crazyflie_shows.plan_show --yaml config/crazyflies.yaml
```

## Layout

```
crazyflie_shows/
├── package.xml CMakeLists.txt setup.py setup.cfg   packaging
├── HANDOVER.md      the rig
├── SHOW_GUIDE.md    the show
├── crazyflie_shows/
│   ├── figures.py        slot layouts, easing, trajectory fitting   PURE
│   ├── safety.py         separation / envelope checking             PURE
│   ├── choreography.py   THE SHOW: config, plan, verification       PURE
│   ├── plan_show.py      offline verifier + report      entry point
│   ├── swarm_show.py     the flight script             entry point
│   └── demo_show.py      the original starter show     entry point
├── config/           this package's crazyflies.yaml + motion_capture.yaml
├── launch/           show_launch.py — wraps crazyflie/launch.py
├── scripts/          mocap_diag.sh, mocap_verify.sh
└── data/             trajectory CSVs (new files need a rebuild)
```

The three `PURE` modules import numpy and two classes from
`crazyflie_py.uav_trajectory` and nothing else — no rclpy, no radio, no mocap.
That is what lets the whole show be verified on a laptop.

`config/*.yaml` are passed to `crazyflie/launch.py` as launch arguments, so the
vendored copies are never edited. `server.yaml` and the URDF are **not**
overridable — they always come from the `crazyflie` package.

## Before every hardware flight

Full checklist in `SHOW_GUIDE.md` §5d. The four that bite:

- `sudo ip addr add 141.23.110.162/32 dev wlp131s0f0` — **per session**, and
  again after any Wi-Fi reconnect. The mocap node ignores `interface_ip` and
  always joins multicast on that hardcoded address (HANDOVER §4a). Setting
  `interface_ip` does nothing; do not spend time on it.
- `ros2 run crazyflie scan --address 0xE7E7E7E7NN` for **every** enabled drone —
  one unreachable drone hangs the server silently and forever
- `ss -uanp | grep :1511` → two sockets means a leftover process is starving mocap
- drones ≥1 m apart, each at **its own** `initial_position`. `swarm_show`
  checks this against the live pose and refuses to arm if a drone is more than
  0.25 m off its mark.
