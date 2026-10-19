# STL pursuit-evasion show

5 defenders vs 5 intruders: a colleague's Signal-Temporal-Logic game, flown
on ten Crazyflies. Added 2026-10-13.

| File | What it is |
|---|---|
| `crazyflie_examples/.../stl_pursuit_evasion_game_v3.py` | **The original.** The colleague's work, kept unchanged and still runnable (`ros2 run crazyflie_examples stl_pursuit_evasion_game_v3`). Arrives over Slack, not git. |
| `crazyflie_shows/stl_game.py` | The algorithm, copied: constants and helpers line for line, `main()` split into `play()` (no ROS) and `fly()` (the command sequence). Plus `verify()`, which is ours. |
| `crazyflie_shows/stl_show.py` | Flies it under this package's contract (WRITING-A-SHOW.md section 1). |
| `crazyflie_shows/plan_stl.py` | Offline check of any seed, and the drift check against the original. |
| `missions/stl.yaml` | The console's mission window. |

Nothing in `crazyflie_shows` imports `crazyflie_examples`. The original is
only ever read as a file, by `plan_stl --against`.

## Why the flight can be checked before takeoff

The game never reads mocap. Every setpoint comes from its own model, and its
only randomness is one generator. So **a seed fixes the whole flight**:
`stl_game.play(seed)` computes every `goTo`, `land` and LED cue up front
(~1 s), `verify()` checks that table, and `fly()` sends that same table. A
seed that `plan_stl` passes is the flight `stl_show` flies. Every run prints
its seed, and `seed:=N` replays it.

If a later version closes the loop on mocap, this property is gone, and with
it the pre-flight proof. The port would then need the escort's approach
(clamp at flight time, ESCORT.md).

## What the show adds, and what it deliberately does not

Added, all outside the algorithm:
- `verify()` before arming: the takeoff columns plus the 3 s start move from
  each drone's `initial_position` to its start slot (closed form, exact), and
  every tick of the game and the return leg, against `min_sep` (default
  0.15 m). A BLOCKED intruder stays in the check for its whole 5 s descent.
- `wait_for_mocap` and `check_placement` (each drone on its own mark),
  `report_supervisor`, and `dry_run`.
- `take_signals` / `abort_land`, so Ctrl-C or the console's Stop lands all
  ten. The E-STOP is still the answer to a drone out of control.
- Ticks paced to `t0 + k*DT` instead of `DT` after the last sleep. That
  changes nothing in a stub and stops latency accumulating on hardware.

Not changed:
- The game's 4 m x 4 m square centred on the origin. It does not use
  `safety.ARENA_*`.
- The colours, including green for REACHED, which is also the rig's "ready"
  colour.
- `goTo` streaming at 20 Hz, rather than `cmdFullState`. It is what flew on
  hardware.

## Measured 2026-10-13 (v3, current yaml)

`plan_stl` on seeds 0-19:
- **The port is identical to the original**: every command, value and time
  on all 20 seeds (`--against`).
- **Every seed: 5 BLOCKED, 0 REACHED**, game 9.0-10.5 s, return ~4.3 s.
- **Closest approach among drones at cruise height: 0.18-0.20 m.** That is
  the design (BLOCK_RADIUS = COLLISION_RADIUS = 0.20 m).
- **0.108 m on every seed, cf5 to cf13, about 3.5 s in, so the default
  refuses.** cf13 is BLOCKED at tick 64 and told to land. Its descent is a
  7th-order polynomial, so 0.35 s later it is still at 0.999 m, and cf5 passes
  it at 10.8 cm. The cause is in the algorithm: once an intruder is out of
  the game, defenders keep only `INTR_KEEP_OUT = 0.04 m` from it. That is a
  question for the colleague, not something to patch here. To fly v3 anyway,
  pass `min_sep:=0.10` knowingly.
- **The start move depends on the marks.** On the yaml marks as committed,
  cf10 and cf11 pass 0.043 m apart, and cf10's 3.14 m leg peaks at
  2.29 m/s. If the drones really sit elsewhere, sync the marks
  (`server:=False`, then `scripts/sync_initial_positions.py`) and re-run
  `plan_stl`. `check_placement` refuses a flight whose marks are wrong.
- **Simulator** (`backend:=sim`, seed 0, `min_sep:=0.04`): flew takeoff to
  disarm. `kill -INT` mid-game landed all ten and disarmed (exit 130).
  Hardware has not been flown with `stl_show` yet.

## Running it

```bash
ros2 run crazyflie_shows plan_stl                       # seeds 0-19 on the yaml marks
ros2 run crazyflie_shows stl_show --ros-args -p dry_run:=true
ros2 run crazyflie_shows stl_show --ros-args -p seed:=7 -p min_sep:=0.10
```

All ten drones must be `enabled: true`: cf1/2/4/5/8 defend, cf10-14 intrude
(`defenders:=` / `intruders:=` to change, in slot order). Any other enabled
drone stays on the ground but is still landed by an abort.

## When v4 arrives

1. Save it beside v3 in `crazyflie_examples/crazyflie_examples/`, keeping v3,
   and commit it as received. Add its `setup.cfg` entry if you want to run it
   raw.
2. `diff v3.py v4.py`. The constant block in `stl_game.py` is v3's
   verbatim, so a constant change ports as the same edit. Loop changes go
   into `play()` in the same order of arithmetic.
3. Point `SOURCE_SHA256` / `SOURCE_NAME` in `stl_game.py` at v4
   (`sha256sum v4.py`).
4. `plan_stl --against .../stl_pursuit_evasion_game_v4.py` must print
   *identical*. It diffs every call the two make, on every seed. Until it
   does, the port is not v4. If v4 starts reading mocap or changes how it
   talks to the drones, the stubs in `plan_stl` and `fly()` change too.
5. Re-run `plan_stl` without `--against` for the new separation numbers, then
   `dry_run`, then sim, then hardware, and update the numbers above with a
   date.
