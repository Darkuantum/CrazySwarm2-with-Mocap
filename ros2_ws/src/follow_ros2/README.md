# follow_ros2

ROS 2 adapters that run the person-following autonomy onboard a ModalAI
Starling 2 Max (VOXL 2).

These nodes contain **no autonomy logic**. Everything — the EKF, the standoff
law, the speed governor, the command arbiter, the track supervisor — lives in
`follow_core/` at the repo root, which imports nothing from ROS and is covered
by the test suite there. These nodes convert messages to and from that API.
Keep it that way: logic that lands here stops being testable off-target.

## The distro constraint

| | Distro | Why |
|---|---|---|
| Target (VOXL 2) | **Foxy** | ships as `apt-get install voxl-ros2-foxy`; newer needs Docker |
| Dev machine | **Humble** | `/opt/ros/humble` |

The code must build and run on both, so it sticks to plain
`declare_parameter(name, default)` / `get_parameter(name).value` and to message
types present in Foxy. Where a choice was made for Foxy compatibility there is a
comment saying so — please don't "modernise" those and break the target.

**Verified here:** builds under `colcon` on Humble, all four nodes start, and
the full pipeline passes an end-to-end message test. **Not verified:** Foxy, and
any hardware at all.

## Nodes

| Node | Subscribes | Publishes |
|---|---|---|
| `follow_node` | `/qvio` (Odometry), `/uwb/range`, `/person/bearing`, `/follow/command` | `/follow/goal_pose`, `/follow/target_estimate`, `/follow/status`, `/follow/markers` |
| `detector_bridge` | detector boxes | `/person/bearing` |
| `uwb_node` | serial from the band | `/uwb/range`, `/follow/command` |
| `mapper_bridge` | `/follow/goal_pose` | goal to `voxl-mapper`; HOLD if no plan |

### `mapper_bridge` is not optional

PX4 Collision Prevention is Position-mode only and **inactive in Offboard**,
which is the mode an autonomous follower flies in. Obstacle avoidance therefore
has to come from `voxl-mapper`'s ESDF, reached by setting `voxl-vision-hub`'s
`offboard_mode` to `trajectory`. If no valid plan exists, this node publishes
HOLD rather than let the vehicle proceed on an unchecked straight line.

### `uwb_node` and the E-stop

The band's E-stop packet is translated to a `FollowCommand` and published the
instant it is parsed — never batched, never behind a queue. The wire protocol
parser (`uwb_protocol.py`) is deliberately free of ROS imports so it can be
unit-tested off-target; `tests/test_wire_protocol.py` at the repo root compiles
the ESP32 firmware's own `crc8()` and checks all three implementations agree.

## Build and run

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install
source install/setup.bash
export PYTHONPATH=/path/to/follow_drone:$PYTHONPATH   # so follow_core resolves

ros2 launch follow_ros2 follow.launch.py
```

Run it against PX4 SITL before hardware. On the vehicle, check
`config/follow_params.yaml` — in particular the camera extrinsic and the
intrinsics in `detector_bridge`, which ship as 640x480 placeholders and **must**
be replaced with a real calibration.

## Two things that will bite you

**Commands must be re-asserted.** The arbiter expires anything older than
`command_timeout_s` (default 2 s) and falls back to HOLD, so a ground station
that goes quiet cannot leave the vehicle following. Publish `FollowCommand` at
2 Hz or better.

**Check the camera down-pitch.** At 2.5 m standoff and 2.0 m height the operator
sits ~39° below horizontal. With a 50° vertical FOV, a 15° down-pitch puts them
on the bottom edge of frame and bearings drop out constantly while the logs look
nominal. See `docs/ARCHITECTURE.md`.
