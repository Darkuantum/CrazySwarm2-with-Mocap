# Provenance: what is inherited, and what is ours

Most of this workspace is other people's code, copied in unchanged. This page says who
owns each layer, measured by comparing each vendored tree with the exact upstream
commit it came from (2026-10-19). Use it to judge where a refactor is cheap and where
it means maintaining a divergent copy of upstream.

## The layers

| Layer | Owner | What it fixes in place |
|---|---|---|
| Drone firmware, Crazyradio, radio link | **Bitcraze** | Onboard estimator and controllers, high-level vs streamed commands and their priority, the supervisor latch after an E-STOP, the 1 s "host gone" timeout. Stock firmware; none of it is in this repo. |
| Motive, NatNet | **OptiTrack** | How poses leave the Motive PC. |
| ROS 2 architecture | **IMRCLab**: [crazyswarm2](https://github.com/IMRCLab/crazyswarm2), [motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking) | Package layout; Motive → `/poses` → server → poses broadcast to every drone; the `crazyflie_py` API; the three config files; the simulator. |
| Rig tooling | **AI-DA-STC** team repo ([CrazySwarm2-with-Mocap](https://github.com/AI-DA-STC/CrazySwarm2-with-Mocap)) | Preflight GUI, three examples, RViz/Foxglove in the launch, live parameter pushes, LED on connect. |
| Hardening and the demos | **This fork** | Server and radio-library robustness, Motive auto-discovery, the mocap driver patches, `crazyflie_shows`, `console/`. |

## The measurements

Files compared with line endings ignored. "Diff lines" counts added plus removed lines.

| Tree | Upstream commit | Identical | Changed | Added here |
|---|---|---|---|---|
| `src/crazyswarm2` (excluding `crazyflie/deps`) | IMRCLab `ae23edc` | 374 | 18 | `crazyflie_shows/`, `arena.yaml`, 6 AI-DA-STC files, 1 STL example |
| `src/crazyswarm2/crazyflie/deps` (crazyflie_cpp, crazyflie-link-cpp) | submodules of `ae23edc` | 528 | 8 | none |
| `src/motion_capture_tracking` | IMRCLab `ros2@64d3af2`, libmotioncapture `24321e4` | 1828 | 4 | 3 patches + `VENDORED.md` |
| `src/natnet_ros2` | L2S-lab `883b095` | 30 | 2 | the vendored NatNet SDK |

Who changed the 26 changed Crazyswarm2 files (diff lines):

| Area | AI-DA-STC vs IMRCLab | This fork vs AI-DA-STC |
|---|---|---|
| `crazyflie_server.cpp` | 216 | 546 |
| `launch.py` | 171 | 29 |
| `crazyflie_cpp` + `crazyflie-link-cpp` | 62 | 655 |
| Configs (`crazyflies.yaml`, `server.yaml`, `motion_capture.yaml`, rviz) | 139 | 172 |
| Examples, `crazyflie_py`, simulator, build files, `UploadTrajectory.srv` | 251 | 79 |

The fork's changes are itemised, with reasons, in [SERVER-CHANGES.md](SERVER-CHANGES.md).
The mocap driver's patches are in
[`src/motion_capture_tracking/VENDORED.md`](../src/motion_capture_tracking/VENDORED.md).

Wholly our own code (lines): the escort, 4,209; the shared show safety code (`abort`,
`preflight`, `safety`), 893; carousel and constellation, 3,108; STL, 1,721; `console/`,
10,702; `scripts/`, 4,328.

## What this means for refactoring

- **The inherited code is not ours to restructure** (about 99% of its files are unchanged). Every change there is a merge
  conflict at the next upstream update, so it should stay small and be offered back
  ([CONTRIBUTING-UPSTREAM.md](CONTRIBUTING-UPSTREAM.md)). The server patches are already
  the largest divergence; the one open refactor there (per-drone callback groups, see
  [CLAUDE.md](../CLAUDE.md)) is a concurrency change to the binary that owns the radio.
- **The data flow is inherited too.** Mocap → `/poses` → server → radio is IMRCLab's
  design, and the Bitcraze firmware sets the command model. Changing either means
  leaving Crazyswarm2, not refactoring it.
- **Our own code is where refactoring is free:** `crazyflie_shows`, `console/` and
  `scripts/`. The known knot is that `escort_show` imports helpers from the
  carousel and constellation modules (`swarm_show`, `constellation`,
  `constellation_show`, `plan_show`), so the escort cannot ship without them. Moving
  those helpers into a shared module would decouple the shows.

## Regenerating the numbers

```bash
git clone https://github.com/IMRCLab/crazyswarm2.git /tmp/up && cd /tmp/up
git checkout ae23edc && git submodule update --init --recursive
cd - >/dev/null
git -C /tmp/up ls-files --recurse-submodules | while read f; do
  [ -e "src/crazyswarm2/$f" ] || { echo "missing $f"; continue; }
  diff -q --strip-trailing-cr "/tmp/up/$f" "src/crazyswarm2/$f" >/dev/null || echo "changed $f"
done
```

Same loop for `motion_capture_tracking` (`ros2@64d3af2`) and `natnet_ros2` (`883b095`).
Split by author with `git show upstream/main:src/crazyswarm2/<file>`.
