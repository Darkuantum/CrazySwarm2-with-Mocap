# follow-drone

A person-following drone that takes commands from a tablet and a wearable
gesture band, and plans around obstacles with its own onboard perception.

- **Primary aircraft** — ModalAI Starling 2 Max (VOXL 2 / QRB5165, PX4 on the
  sensors DSP, 566 g take-off, 500 g payload, C29 camera config with ToF)
- **Prototyping rig** — Crazyflie 2.1 Brushless in an OptiTrack volume
- **Target localization** — hybrid: UWB two-way ranging for range and identity,
  onboard vision for bearing
- **Window** — 15 weeks, 8 Sep – 20 Dec 2026, one engineer

Full specification, requirements and schedule:
<https://claude.ai/code/artifact/1895f532-fcd4-446a-801d-701c99c6a1de>

## Layout

| Path | What it is | Verified? |
|---|---|---|
| `follow_core/` | All the logic. Pure Python, no ROS, no hardware imports. | **72 unit tests, all passing** |
| `tools/sim.py` | Closed-loop simulation, scores against FR-1 | **4/4 scenarios pass** |
| `tests/` | pytest suite, including the PRD acceptance criteria | — |
| `ros2_ws/src/` | ROS 2 nodes for VOXL 2 (thin adapters over `follow_core`) | **builds on Humble; 4/4 nodes start; end-to-end message test passes** |
| `crazyflie/` | Drop-in package for the OptiTrack rig | compiles; not built (needs the other workspace) |
| `firmware/band/` | ESP32 wearable: gestures, hardware E-stop, UWB tag | **not compiled** — no PlatformIO here |
| `config/` | PX4 params, voxl-vision-hub, voxl-mapper, mocap | not applied to hardware |
| `docs/` | `ARCHITECTURE.md`, `BRINGUP.md`, `SAFETY.md` | — |

## Quick start

```bash
python3 -m pytest tests/ -q          # 72 tests, ~1.6 s
python3 tools/sim.py --scenario straight
python3 tools/sim.py --scenario occlude --plot /tmp/follow.png
```

Current simulation results (20 s, seed 42, FR-1 limits 0.35 m mean / 0.75 m p95):

| Scenario | mean err | p95 err | in frame | verdict |
|---|---|---|---|---|
| `straight` — walk, 90° turn, walk | 0.044 m | 0.092 m | 100 % | PASS |
| `eight` — figure-eight, course reverses | 0.101 m | 0.216 m | 100 % | PASS |
| `fast` — 1.4 m/s, the FR-1 upper bound | 0.044 m | 0.094 m | 100 % | PASS |
| `occlude` — walks behind two pillars | 0.041 m | 0.090 m | 81 % | PASS |

`occlude` is the one that justifies the architecture: vision is unavailable 19 %
of the time and the UWB range keeps the lock, with the target never declared
lost.

## Read this before using `follow_core`

**Every command source must heartbeat.** The arbiter expires any command older
than `command_timeout_s` (default 2 s) and falls back to HOLD, so a ground
station that goes quiet cannot leave the vehicle following. Send FOLLOW once and
stop, and the vehicle will hold — that is correct, not a bug. The tablet, the
band and the simulator all re-assert at 2 Hz.

**The goal is not a path.** `follow_core` says where the vehicle should be.
`voxl-mapper` owns how to get there without hitting anything, because PX4's
collision prevention is inactive in Offboard mode. See `docs/ARCHITECTURE.md`.

**Check the camera down-pitch against your mount.** At 2.5 m standoff and 2.0 m
height the operator is ~39° below horizontal; a 15° pitch puts them on the
bottom edge of a 50° vertical FOV and vision drops out constantly while the logs
look nominal.

## Status — what is and is not proven

Nothing in this repository has flown. What has actually been executed here:

- `follow_core` — 60 unit tests pass, including analytic-vs-numerical Jacobian
  checks on both measurement models, covariance symmetry/PSD under sustained
  fusion, and the identity-gating behaviour behind FR-2.
- `tools/sim.py` — all four scenarios meet FR-1 across four random seeds.
- ROS 2 packages — `colcon build` succeeds on Humble, all four nodes start, and
  a synthetic end-to-end run (odometry + UWB range + bearing + command in) puts
  the goal exactly 2.5 m behind the operator with 117/117 measurements accepted.
  No hardware, and the target runs **Foxy** while this machine runs **Humble**.
- Band firmware — written, never compiled. The DW3220 driver is deliberately
  stubbed with the required vendor calls named.
- Configs — never applied to a vehicle. Treat every value as a starting point to
  be diffed against the on-device defaults.

Three bugs were found and fixed by the tests, and are worth knowing about
because each was a plausible in-flight failure:

1. A gesture STOP could be overridden by an *older* tablet FOLLOW still inside
   its freshness window — a stop that un-stops itself.
2. The speed governor discarded a deliberate zero ceiling, so a HOLD setpoint
   came back with a 3 m/s velocity budget.
3. The estimator diverged silently whenever vision dropped out, because
   range-only updates are consistent with any point on the range sphere and
   produce no rejections while doing it.
4. The Crazyflie node's `SetpointGuard` clamped minimum standoff *before* its
   speed and acceleration slew, and the slew then interpolated back toward the
   previous setpoint — so whenever the operator walked toward a drone holding
   station, the clamp silently did nothing. The clamp is now re-asserted after
   the slew; remove it and `tests/test_setpoint_guard.py` fails immediately.

## First things to do on hardware

1. Start CAAS registration and settle the educational-vs-commercial
   classification. Registration applies **indoors** above 250 g.
2. Weigh the actual aircraft. ModalAI's datasheet says 566 g; the launch press
   release says 500 g.
3. Confirm C28 vs C29 — you want the ToF. Stereo point clouds have been measured
   as too sparse indoors for a usable cost map.
4. Assert the TFLite delegate actually binds. Delegates **fail soft**: the
   server silently runs on CPU. Use MobileNet-SSD (7.72 ms via NNAPI), not the
   YOLOv8/v11 models that ship with the SDK.
5. Run `pytest` and `tools/sim.py` before every flight session.
