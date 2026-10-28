# Provenance: follow-drone

The person-following drone prototype (tablet + wearable band, UWB ranging, onboard
perception; Crazyflie first, Starling 2 Max / PX4 later). It was developed
out-of-tree at `~/near-intern/follow-drone`, a local repo that was never pushed
anywhere, and was folded in here on 2026-10-28 so that it survives the laptop
hand-back. Archive only: nothing here is built, launched or maintained.

## History

The original five commits (8 Sep to 28 Oct 2026) are kept as the tag
**`archive/follow-drone-e868409`**. This folder is that tag's tree, unchanged, plus
two files added on the way in: this one and `COLCON_IGNORE`.

```bash
git log archive/follow-drone-e868409                              # the original history
git diff archive/follow-drone-e868409 HEAD:prototypes/follow-drone --stat   # only the added files
```

## What this fork inherited from it

Numbers and one lesson, not code. `crazyflie_shows/escort.py` and
`docs/details/escort-design-notes.md` cite each one where it is used:

- the 1.5 m minimum standoff, the 0.6 m/s speed cap and the 4 Hz mocap
  velocity-filter corner (from `crazyflie_follow`'s `mocap_target`);
- the 2.5 m arena radius and 2.0 m ceiling, since replaced by the measured
  `src/crazyswarm2/crazyflie/config/arena.yaml`;
- re-assert the standoff clamp *after* the slew. The prototype shipped that
  bug; `tests/test_setpoint_guard.py` here pins the fix.

Nothing else (`follow_core`, the band firmware and wire protocol, the ROS 2
packages, the PX4/VOXL configs) exists anywhere else.

## Using it in place

```bash
cd prototypes/follow-drone
python3 -m pytest -q tests        # 72 passed on 2026-10-28
```

`COLCON_IGNORE` keeps its three ROS 2 packages (`follow_msgs`, `follow_ros2`,
`crazyflie_follow`) out of this workspace's root-level `colcon build`. To build
them, copy them out or build `ros2_ws/` on its own.

`crazyflie_follow` finds `follow_core` by walking up from its own file, which
still works at this path. Its last fallback, the fixed `~/near-intern/follow-drone`,
and the docs' `~/near-intern/...` paths describe the old laptop and no longer exist.
