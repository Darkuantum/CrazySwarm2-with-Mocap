# The mission console — the GUI alternative to typing `ros2` commands

`console/` is a browser front end for this rig. It is **optional**, it is
**removable**, and it is deliberately *not* a second way of talking to the
drones: every button runs the same `ros2` or shell command you could type, shows
you that exact command before and while it runs, and keeps a log you can export
as a shell script. The stated goal is that an operator **learns the commands
from the GUI and eventually stops needing it**. This document therefore gives,
for everything the console does, both the GUI path and the terminal equivalent.

**Who owns what.** This file is the operator's map: GUI action → exact command,
and the limits of the GUI. [`console/README.md`](../console/README.md) owns the
internals (module layout, data flow, the HTTP API, how to extend each file) —
read it before editing the console. CLAUDE.md owns the rig's gotchas; the health
diagram (section 4) is those gotchas made executable. Where this file and the
argv the console displays disagree, **the displayed argv is the truth** — it is
what runs — and this file is the bug.

**How this was written, and what was not verified.** Every command in section 3
was read out of the `build` function of its action in
`console/mission_console/catalog.py`, or out of `server.py` / `health.py` /
`procs.py` for the things that do not go through the catalog. **Nothing here was
executed against hardware or a live console.** Points that rest on someone
else's measurement rather than a read of the code are marked where they appear.

Contents: [1 What it is](#1-what-it-is-and-that-it-is-removable) ·
[2 Starting it](#2-starting-it) ·
[3 GUI action → command](#3-gui-action--terminal-command-the-centrepiece) ·
[4 The health diagram](#4-the-health-diagram) ·
[5 Rules that bite](#5-rules-that-bite) ·
[6 Honest limits](#6-honest-limits) ·
[7 Extending it](#7-extending-it)

---

## 1. What it is, and that it is removable

* **Plain Python 3**: the stdlib HTTP server plus PyYAML (already a dependency),
  driving the `ros2` CLI as child processes, with a single-page front end in
  `console/mission_console/static/`. No rebuild, no colcon package, no new
  dependency to install.
* **Removable.** `rm -rf console/` removes the feature and changes nothing else.
  Nothing in `src/` imports it; `launch.py`, `CMakeLists.txt` and `setup.sh` do
  not know it exists. The only things pointing at it are prose (README section 8,
  CLAUDE.md, this file). **Keep it that way** — do not make anything in `src/`
  depend on it, and do not turn it into a colcon package.
* **`ros2` CLI, never in-process rclpy.** Two reasons, both from this rig's
  record: a fresh rclpy process can stall forever in DDS discovery (the same
  reason `scripts/led.sh` drives the CLI), and a CLI command is something the
  operator can copy into a terminal. The CLI's long-running daemon keeps the ROS
  graph warm.
* **Not a safety layer.** It will let you run a show that is not safe to run.
  The refusals that matter live in the shows themselves
  (`crazyflie_shows/preflight.py`) and in `scripts/sync_initial_positions.py`.
  See [6.4](#64-other-limits).

## 2. Starting it

```bash
./console/run.sh                  # http://localhost:8077, opens a browser
./console/run.sh --port 9000
./console/run.sh --no-browser
./console/run.sh --host 0.0.0.0   # reachable from the lab LAN — read the warning
```

`run.sh` exists so the console's children inherit exactly the environment a
terminal on this rig would have — which is why the commands it shows are
commands you can paste. In order, it: strips conda/venv from `PATH` and
`PYTHONPATH` (ROS runs nodes with `/usr/bin/python3`); sources
`/opt/ros/$ROS_DISTRO/setup.bash` **under `set +u`** (ROS's `setup.bash` reads
the unbound `AMENT_TRACE_SETUP_FILES` and aborts under `set -u`); sources
`$REPO/install/setup.bash` so the **vendored** `motion_capture_tracking` wins
over the apt one; then `exec /usr/bin/python3 -m mission_console`. Each step
warns rather than aborting if it cannot be done, and the System health diagram
then reports the gap.

**There is no authentication.** It binds `127.0.0.1` by default. With
`--host 0.0.0.0`, anyone who can reach the port can arm and fly the drones. Use
an SSH tunnel instead:

```bash
ssh -L 8077:localhost:8077 <rig-host>
```

The ROS domain is shared with the lab (domain 0 by default; a colleague's
`/all/takeoff` is visible here). The console uses whatever `ROS_DOMAIN_ID` it
inherited and shows it in the nav bar — if that does not match the shell that
launched the stack, the console sees an empty graph.

Ctrl-C on the console sends SIGINT to every long-running process it started, as
closing the terminal those were launched from would; a second signal exits
immediately.

### The tabs

| Tab (keys `1`…`5`) | What it is |
|---|---|
| **Dashboard** | Per-drone tiles (supervisor state in words, battery, link), a "needs attention" row, a **stepper** (before the server / fleet positions / bring it up / check / fly) that follows the rig and shows only the current step's buttons, your **pinned** shortcuts, **More** (every action, ranked by how often you use it, ☆ to pin), a **Missions** row that opens each mission's own window, and the **only process view**: everything the console started down the left, the selected one's output on the right with Stop, Kill, Copy, follow, and a stdin line. (The separate Processes tab was folded in here on 2026-10-09.) Every button is a catalog action; what it shows is the real argv. |
| **System health** | The probe dependency graph — [section 4](#4-the-health-diagram). |
| **Control** | The reference: one verbose card per catalog action, grouped Launch, Preflight, Fleet position, Flight, Commands, Parameters, Recovery. Each card carries a `why` and a "how to read this command line". |
| **Config** | `crazyflies.yaml` as a table, plus a raw editor for each of four config files — [5.3](#53-yaml-writes-are-text-surgical). |
| **Command log** | The session so far; "Download as a shell script" exports it with a prelude. **Usage across sessions** (collapsed) shows what has actually been run, from where, and how often — the data the Dashboard's More menu is ranked by. |

**Mission windows** — one per demo, each defined by a `missions/<name>.yaml` in
the show's package and opened from the Dashboard's Missions row — are documented
in [MISSIONS.md](MISSIONS.md): a live 3D view (through foxglove_bridge), the
show's prompts as buttons, its keyboard teleop as a key pad, status pulled from
its output, and an abort that goes through the show's own landing.

**The usage log.** Every run (with where it was started from), stop, e-stop,
tab, palette use and mission is appended to `console/usage/usage.jsonl`
(git-ignored; stdin is recorded only as `enter` / `q` / `text`, never its
content). Pins live next to it in `console/usage/dashboard.json`. Both are plain
files — `cat` them, or ask Claude to read them before rearranging the Dashboard.

`Ctrl-K` or `/` opens a palette over every action, health check, config file,
mission and tab. `Enter` runs the highlighted entry through the same confirm step as the
button; `Shift-Enter` opens its card instead. `r` re-runs every probe, `?` lists
the keys, `Esc` closes whatever is open. **E-STOP deliberately has no keyboard
shortcut** — a stray keypress must never cut the motors — and it is the one
button with no confirmation step.

---

## 3. GUI action → terminal command (the centrepiece)

Run the terminal equivalents from the repo root, in a shell that has done
`source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash`, with no
conda active.

Two columns need defining:

* **Needs** is the action's `requires`. `server stopped` means the crazyflie
  server must **not** be running, because only one process may own a Crazyradio;
  `server running` means the stack is up. **This is a warning, not a lock**: an
  unmet precondition greys the Dashboard button and adds a red line to the
  Control card and the confirm dialog, but the backend will still run what you
  confirm. Treat it as advice that is usually right.
* **Confirm** marks actions the console puts behind a modal that shows the argv
  first. That happens whenever an action is tagged dangerous (moves drones, cuts
  motors) *or* a precondition is unmet — so some unmarked rows will still prompt
  when the stack is in the wrong state.

Values in `<angle brackets>` are card fields; the defaults shown are the form
defaults in `catalog.py`. Empty number fields fall back to the default rather
than emitting `{height: }`, which is invalid YAML and would fail in the parser.

### Launch and recovery

| GUI path | Needs | Exact command | Notes |
|---|---|---|---|
| Launch → **Start the stack** | no stack running (server **or** mocap-only) | `ros2 launch crazyflie launch.py backend:=cpp rviz:=True preflight:=True foxglove:=True teleop:=True mocap:=True gui:=False debug:=False` | One `name:=value` per card field, always all eight. `mocap_hostname:=<ip>` is appended **only** when that field is non-empty. `backend` is `cpp`, `cflib` or `sim`. These are launch arguments, not shell syntax — no spaces around `:=`. |
| Launch → **Start mocap only (no server)** (Dashboard step 1) | no stack running | `ros2 launch crazyflie launch.py server:=False rviz:=True foxglove:=True` (+ `mocap_hostname:=<ip>` if set) | Mocap, RViz and the bridge, **no server**: nothing owns the radio, nothing can arm. The state the position sync wants. |
| Launch → **Restart with the server** (Dashboard steps 2 and 3) | any · confirm | `bash -c './console/stop_stack.sh && exec ros2 launch crazyflie launch.py <the eight Start-the-stack arguments>'` | Stops whatever stack runs — verifying UDP 1511 is free — then starts the full stack, so the server reads the `crazyflies.yaml` just synced. One command, so there is no window in which a second launch starts a second mocap node on 1511. `exec` makes Stop on this process signal the launch itself. |
| Launch → **Start the simulator stack** | no stack running | `ros2 launch crazyflie launch.py backend:=sim` | Needs `./scripts/setup_sim_firmware.sh` once. Flight scripts then need `use_sim_time:=true` (below). |
| Launch → **Stop the stack** | any · confirm | `./console/stop_stack.sh` — with the "skip to SIGKILL" field set: `./console/stop_stack.sh --hard` | Signals each stack process directly (launcher first, then server, mocap node, preflight GUI, foxglove bridge, RViz), escalates SIGINT → SIGTERM → SIGKILL, and then **verifies UDP 1511 is released**. That verification is the whole point: a bare Ctrl-C on `ros2 launch` routinely leaves the mocap node alive and holding 1511, and the *next* launch starves silently. |
| Launch → **Rebuild the workspace** | any | `./scripts/build.sh` or `./scripts/build.sh <package>` | Restart the console afterwards — [5.1](#51-restart-the-console-after-changing-its-code). |
| Recovery → **Kill leftover mocap nodes** | any · confirm | `bash -c 'pkill -f motion_capture_tracking_node; sleep 1; pgrep -af motion_capture_tracking \|\| echo "clear"'` | Killing a `ros2 run` wrapper does not kill the node binary. |
| Recovery → **Kill a hung stack** | any · confirm | `bash -c 'pkill -f "ros2 launch crazyflie"; pkill -f crazyflie_server; pkill -f motion_capture_tracking_node; sleep 1; pgrep -af "crazyflie_server\|motion_capture_tracking" \|\| echo clear'` | For the silent mid-connect hang and for a wedged upload: both ignore SIGINT and need SIGKILL. |
| Recovery → **Restart the ros2 daemon** | any | `bash -c 'ros2 daemon stop; ros2 daemon start; ros2 daemon status'` | When `ros2 node list` is empty while nodes are plainly running. |
| Recovery → **Show the ROS environment** | any | `bash -c 'echo "ROS_DISTRO=$ROS_DISTRO"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (default)}"; echo "RMW=${RMW_IMPLEMENTATION:-default}"; echo "python3=$(command -v python3)"; echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH" \| tr ":" "\n"'` | Most "I see no topics" is a domain mismatch, an unsourced overlay or a conda python. |

### Preflight — read-only; none of these move a drone

| GUI path | Needs | Exact command |
|---|---|---|
| Preflight → **Scan every enabled drone** (also Dashboard "Scan the fleet") | server stopped | `bash -c 'for a in <enabled addresses from crazyflies.yaml>; do echo "== $a"; ros2 run crazyflie scan --address $a; done'` — the address list is substituted at catalog-build time from the yaml. **Terminal equivalent, and the better one:** `./scripts/scan_fleet.sh`, which derives the list itself and prints a GO / NO-GO verdict. `--list` prints only what would be scanned, `--all` includes `enabled: false` drones, `--yaml PATH` takes a one-off fleet file. |
| System health → **Scan the fleet** button (`POST /api/scan_fleet`) | server stopped | The same `ros2 run crazyflie scan --address <addr>` per enabled address, 25 s timeout each. **This path checks one thing the script does not:** that the answering URI's datarate matches the datarate in the yaml URI, and says so explicitly if it does not. `scan_fleet.sh` leaves that comparison to you ("check each answer's datarate matches its uri"). A datarate mismatch hangs the server forever. |
| Preflight → **Scan one address** | server stopped | `ros2 run crazyflie scan --address <0xE7E7E7E7xx>` — the dropdown lists the enabled addresses plus the factory address `0xE7E7E7E7E7`, for a drone whose address you do not know. |
| Preflight → **Is the Crazyradio plugged in?** | any | `bash -c 'lsusb \| grep -iE "crazyradio\|bitcraze\|1915:\|35d2:" \|\| echo "no Crazyradio on USB"'` |
| Preflight → **Is UDP 1511 clear?** | any | `bash -c 'ss -uanp \| grep ":1511" \|\| echo "nothing bound to 1511 (expected when the mocap node is not running)"'` — **two sockets here is the confirmed cause** of "mocap suddenly died": with two `SO_REUSEPORT` sockets the kernel hands each NatNet datagram to only one of them, and the mocap node starves silently inside `connect()`. |
| Preflight → **Leftover mocap processes?** | any | `bash -c 'pgrep -af motion_capture_tracking \|\| echo "none (good)"'` — a leftover makes the next connect abort with SIGABRT. |
| Preflight → **Which motion_capture_tracking resolves?** | any | `ros2 pkg prefix motion_capture_tracking` — must print this workspace's `install/`. `/opt/ros/...` means the apt 1.0.9 driver is shadowing the vendored one, and that release hard-codes a foreign interface IP, so `/poses` stays silent forever. |
| Preflight → **List running nodes** | any | `ros2 node list` |
| Preflight → **Are the /all/\* services up?** | any | `bash -c 'ros2 service list \| grep -E "/all/\|takeoff\|land\|go_to\|arm\|emergency"'` — server alive but no `/all/takeoff` means it is blocked mid-connect on an unreachable drone. |
| Preflight → **Measure /poses rate** | any (streams until Stop) | `ros2 topic hz /poses --window 50` — expect ≈50 Hz, the Motive streaming rate. |
| Preflight → **Who publishes /poses?** | any | `ros2 topic info /poses --verbose` — 0 publishers is a local failure, not a Motive one. |
| Preflight → **Echo one onboard pose** | any | `ros2 topic echo /<drone>/pose --once` |
| Preflight → **Echo one status message** | any | `ros2 topic echo /<drone>/status --once` — battery, supervisor bits, link health. "Won't arm" is usually answered here. |
| Preflight → **Echo link statistics** | any | `ros2 topic echo /<drone>/connection_statistics --once` |
| Preflight → **Read a battery over the radio** | server stopped | `ros2 run crazyflie battery --uri <that drone's uri from crazyflies.yaml>` — the `crazyflie_tools` binaries talk to the dongle directly, so the server must be stopped. |
| Preflight → **Run a ground check** | any | `ros2 run <package> <script>` — the dropdown holds only scripts that never command a drone (name starts `plan_`, ends `.sh`, or is `color_led` / `set_param`), so there is no "area clear" prompt. This is where `plan_show`, `plan_constellation` and `plan_escort` live. |

### Fleet position — Dashboard step 2, "Fleet positions"

| GUI path | Needs | Exact command |
|---|---|---|
| Fleet position → **Preview initial_position sync** | mocap running | `python3 scripts/sync_initial_positions.py --dry-run` |
| Fleet position → **Apply initial_position sync** | mocap running · confirm | `python3 scripts/sync_initial_positions.py --yes` |

The order is forced by two facts: the script reads `/poses`, so **mocap must be
up**; and the server reads `crazyflies.yaml` **only when it starts**, so it must
start *after* the sync. Hence: **Start mocap only** → Preview → Apply →
**Restart with the server**. Syncing with the full stack up works too, but the
running server keeps the old marks until it is restarted — which is the same
Restart button. The precondition is a warning (the button dims and says why),
not a lock.
The script rewrites only the `initial_position` triples in `crazyflies.yaml`,
comments preserved, and refuses on its own safety rules (an enabled drone not
streamed, a drone moving, above 0.5 m, or two drones under 1 m apart). `--force`
exists on the CLI; **the console never passes it**. Restart the server
afterwards — the yaml is read only at launch.

### Flight — these move drones

| GUI path | Needs | Exact command |
|---|---|---|
| Flight → **Run a flight script** (Dashboard "Fly") | server running · confirm | `ros2 run <package> <script>`, plus `--ros-args -p <name>:=<value>` for each entry in the ROS-parameters field. With the "simulation clock" selector set to `yes`, `use_sim_time:=true` is inserted as the **first** `-p`. |
| Commands → **E-STOP (all drones)** | server running · confirm | `ros2 service call /all/emergency std_srvs/srv/Empty {}` |
| Commands → **Arm all** | server running · confirm | `ros2 service call /all/arm crazyflie_interfaces/srv/Arm '{arm: true}'` |
| Commands → **Disarm all** | server running | `ros2 service call /all/arm crazyflie_interfaces/srv/Arm '{arm: false}'` |
| Commands → **Takeoff (all)** | server running · confirm | `ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff '{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}'` — height and duration editable. |
| Commands → **Land (all)** (Dashboard "Land all") | server running · confirm | `ros2 service call /all/land crazyflie_interfaces/srv/Land '{group_mask: 0, height: 0.04, duration: {sec: 4, nanosec: 0}}'` — height and duration editable; a longer duration is a gentler descent. |
| Commands → **Takeoff (one drone)** | server running · confirm | `ros2 service call /<drone>/takeoff crazyflie_interfaces/srv/Takeoff '{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}'` — payload **fixed**, not editable on this card. |
| Commands → **Land (one drone)** | server running · confirm | `ros2 service call /<drone>/land crazyflie_interfaces/srv/Land '{group_mask: 0, height: 0.04, duration: {sec: 4, nanosec: 0}}'` — payload fixed. |
| Commands → **Go to (one drone)** | server running · confirm | `ros2 service call /<drone>/go_to crazyflie_interfaces/srv/GoTo '{group_mask: 0, relative: true, goal: {x: 0.0, y: 0.0, z: 1.0}, yaw: 0.0, duration: {sec: 3, nanosec: 0}}'` |

Three things worth carrying away from these rows:

* `builtin_interfaces/Duration` is a nested message, hence
  `duration: {sec: 3, nanosec: 0}`; `group_mask: 0` means all groups.
* The Go-to card's own note flags a real ambiguity: `GoTo.srv` comments `yaw` as
  **degrees** while `crazyflie_py` passes **radians**. Check before trusting a
  large value.
* The ROS-parameters field is validated, not passed through: anything not shaped
  `name:=value` is **rejected loudly** instead of being forwarded. `ros2 run`
  accepts an unknown `-p` without complaint, so a misspelled
  `vip_mode:=manaul` would otherwise launch the escort demo in its *default*
  mode and look like it worked.

### Parameters and LED

| GUI path | Needs | Exact command |
|---|---|---|
| Parameters → **Reset the Kalman filter (all)** | server running | `ros2 param set /crazyflie_server all.params.kalman.resetEstimation 1` |
| Parameters → **Clear the Kalman reset flag** | server running | `ros2 param set /crazyflie_server all.params.kalman.resetEstimation 0` — a reset is **both** writes; the preflight GUI's `r` key does both. |
| Parameters → **List server parameters** | server running | `bash -c 'ros2 param list /crazyflie_server \| head -n 200'` — the list is long because `server.yaml` sets `query_all_values_on_connect: True`. |
| Parameters → **Get a parameter** | server running | `ros2 param get /crazyflie_server <name>` |
| Parameters → **Set any parameter** | server running · confirm | `ros2 param set /crazyflie_server <name> <value>` — `all.params.<group>.<name>` broadcasts; `cfN.params.<group>.<name>` targets one drone. These reach the drones only because the vendored server applies them in an on-set-parameters callback; upstream's `/parameter_events` handler never fires on this rig. |
| Parameters → **Set the Color LED deck** | server running | `./scripts/led.sh <n>` — `0` off, `1` green, `2` red, `3` yellow, `4` blue, `5` purple, `9` white. |

### Console-only — no catalog command, or a different one

| GUI path | What actually happens | Terminal equivalent |
|---|---|---|
| **Header E-STOP** (present on every tab) | `POST /api/estop`: runs the `/all/emergency` call and reports within 3 s — confirmed, or NOT CONFIRMED with the reason. One click, no modal. [5.2](#52-e-stop-is-one-click-with-an-answer-within-3-seconds) | The `ros2 service call` above — but you must judge the outcome yourself: it has no timeout and prints nothing against an unreachable server. The preflight GUI's `e` key is the other independent path. |
| **Config** → table and raw editors | Parse-checked, diffed, backed up, comment-preserving write. [5.3](#53-yaml-writes-are-text-surgical) | Edit the file under `src/crazyswarm2/crazyflie/config/` by hand. |
| **Command log** → Download as a shell script | Writes the session's commands with a prelude: the two `source` lines as comments, then `cd <repo>`. | n/a — this is how a session becomes a runbook. |
| **Dashboard output pane** → Stop | SIGINT to the whole **process group**, escalating SIGINT (6 s) → SIGTERM (4 s) → SIGKILL (2 s); a **flight script** (Dashboard Fly, a mission's main script) gets **15 s** after the SIGINT, so its own abort landing can finish. If it survives SIGKILL the pane says so. | `kill -INT -<pgid>`, or `./console/stop_stack.sh` for the stack. |
| **Dashboard output pane** → Kill | Straight to SIGKILL. | `kill -KILL -<pgid>` |
| **Dashboard output pane** → stdin line (shown while the selected process runs) | Writes what you typed **plus a newline** to the child's pty master. [6.1](#61-operator-paced-and-interactive-runs) | The terminal you would have run it in. |
| **Mission window** → Start | `POST /api/mission/start`: the spec's autostart helpers first, 1.5 s later the script — each `ros2 run <pkg> <exe> --ros-args -p ...`, every argv shown in the confirm step and logged. Refused while another flight script runs. | The same `ros2 run` lines, one terminal each, helpers first. |
| **Mission window** → gate button | Writes the button's `send` line plus a newline to the script's pty (`Continue` = a bare Enter). | Press Enter (or type `q`) in the script's terminal. |
| **Mission window** → Abort & land | **One** SIGINT to the script's process group and nothing after it — no SIGTERM at 6 s. The show's `abort.py` lands where they are. Kill is a separate, confirmed SIGKILL. | Ctrl-C **once** in the script's terminal. |
| **Mission window** → teleop pad / keyboard | Writes the key (no newline) to the helper's pty, re-sent every 90 ms while held; a space on release. | Hold the key in `escort_teleop`'s terminal. |

Anything not in these tables — `trace_cf:=all`, a gamepad — is
a terminal operation. See [6.1](#61-operator-paced-and-interactive-runs) and
[6.2](#62-launch-options-the-card-does-not-expose).

---

## 4. The health diagram

System health is a dependency graph. Each box is an independent probe; a box
downstream of a hard failure turns grey ("blocked upstream") so you look at the
cause rather than the symptom. Clicking a box shows what was measured, the exact
command used, and the fix. This is where CLAUDE.md's Gotchas became executable,
in `health.py`.

```
env.ros ─┐
         ├─► env.overlay ─► cfg.parse ─► cfg.fleet ─┬─► radio.usb ─► radio.drones ─► server.node ─► server.services ─► drone.cfN
env.python ┘                                  │     │
                                              │     └─► mocap.host ─► mocap.pkg ─► mocap.node ─► mocap.poses ─┐
          env.overlay ─────────────────────────────► mocap.pkg                                                 │
                            mocap.port ─────────────► mocap.pkg                      server.services ◄──────────┘
```

| Box | Probe | The known failure it names |
|---|---|---|
| `env.ros` — ROS 2 environment | `ros2` on `PATH`, `$ROS_DISTRO`, `$ROS_DOMAIN_ID` | Console started without ROS sourced; a domain mismatch with the shell that launched the stack. |
| `env.overlay` — Workspace overlay | `install/setup.bash` exists and `install/` is on `AMENT_PREFIX_PATH` | Built but not sourced, so apt packages shadow the vendored, patched ones. |
| `env.python` — Python interpreter | `python3` resolves to `/usr/bin/python3`; no `CONDA_PREFIX` / `VIRTUAL_ENV` | conda shadowing: `No module named '_cffirmware'`, invalid message types. |
| `cfg.parse` — Config files | `crazyflies.yaml` and `motion_capture.yaml` parse | A bad hand edit — the launch cannot start. |
| `cfg.fleet` — Fleet sanity | `configio.validate_crazyflies` | Duplicate addresses, drones under 1 m apart, unknown robot type, two dongles on channels less than 2 apart, a firmware log block over the 26 B budget, malformed `initial_position` or a z not near 0. |
| `radio.usb` — Crazyradio dongle | `lsusb` filtered to Bitcraze IDs | Dongle missing, or a udev/plugdev problem. |
| `radio.drones` — Drones answer radio | `ros2 run crazyflie scan --address <addr>` per enabled drone (25 s each), **only** when you press "Scan the fleet" with the server stopped; also compares the reply's datarate against the URI | The go/no-go check. A dead drone or a datarate mismatch hangs the server forever, silently. Shown as skipped — not red — while the server owns the radio, because `/all/*` coming up already proves every drone answered. |
| `mocap.host` — Motive / NatNet | The same UDP 1510 discovery broadcast `launch.py` uses | Motive not on this LAN segment, not streaming, or firewalled; or the configured host drifted with lab DHCP. A ping to the Motive PC proves nothing — unicast is not multicast. |
| `mocap.port` — UDP 1511 | `ss -uanp \| grep ":1511"` | More than one socket: the confirmed cause of "mocap died". |
| `mocap.pkg` — Mocap driver build | `ros2 pkg prefix motion_capture_tracking` | Resolves under `/opt/ros`: the apt 1.0.9 driver with its hard-coded foreign IP, so `/poses` never arrives. |
| `mocap.node` — motion_capture_tracking | Present in `ros2 node list` | Alive is not streaming — see the next box. |
| `mocap.poses` — /poses stream | `ros2 topic info /poses`, then `ros2 topic hz /poses --window 20` | 0 publishers; publishers but no messages (starved); or under 30 Hz where ≈50 is expected. |
| `server.node` — crazyflie_server | `ros2 node list` | Server not launched, or died. |
| `server.services` — /all/\* services | `ros2 service list` | Server alive but blocked mid-connect on an unreachable drone: the classic silent hang. |
| `drone.cfN` — one tile per enabled drone | `ros2 topic list`, then `ros2 topic echo /cfN/status --once` | Never connected (no `/cfN/takeoff`); no status message; battery below **3.8 V** warn / **3.7 V** fail; unicast latency above **10 ms**; and the supervisor bitfield decoded into words. |

The per-drone tile decodes `supervisor_info` the way the firmware packs it, and
orders the words by what you must clear first: **`E-STOPPED`** (latched locked —
only a battery power cycle clears it), **`CRASHED`**, **`FLIPPED`**, `FLYING`,
`armed`, `ready to arm`. Three things can be wrong at once, and a low battery
must not downgrade an `E-STOPPED` drone to a mere warning, so severity is taken
from the worst of them.

**Two cadences.** A **live sweep every 3 s** re-reads the ROS graph, `/poses`,
and each drone's battery, link and supervisor state. A **full sweep every 30 s**
— and after any action, any config write, the `r` key or "Re-check" — runs
everything, including the NatNet broadcast ping that dominates its cost.
(`console/README.md` gives these as ≈0.14 s and ≈1.7 s; those figures are quoted
from there and were **not** re-measured for this document.)

**Plus an error-signature scan.** Every output line from every process the
console started is matched against `SIGNATURES` in `health.py`, and a match is
attached to the box it belongs to: two dongles under 2 channels apart, no answer
from Motive, the apt driver shadowing, the Qt xcb plugin missing, `_cffirmware`
missing, `NoParameterOverrideProvided`, the server or mocap process dying, a
drone connection timeout, an invalid message type. This is how a failure buried
in a scrolling launch log gets attached to the thing that failed.

Boxes that do not apply to the current run — radio and mocap under
`backend:=sim` — read "not applicable", not red.

---

## 5. Rules that bite

### 5.1 Restart the console after changing its code

`index.html`, `app.js` and `style.css` are re-read on every page load. **The
Python backend is loaded once.** A reloaded page therefore calls routes the
running process does not have, and those answer HTTP 404 with
`{"error": "not found"}`. A teammate reported the web e-stop failing with
"not found"-type wording, and this reproduces it exactly: **the e-stop did not
fire.**

Three guards are in place. `/api/bootstrap` carries `code_changed` (any
`mission_console/*.py` newer than the process start) and the page shows a
restart banner. The page treats a route-miss 404 as "outdated console" rather
than a bare error — matching on the body `"not found"`, because an unknown
process id legitimately 404s as `no such process`. And the e-stop falls back to
`/api/run`, which exists in every version, so it still fires.

Restart it after `./scripts/build.sh` too: a package built after the console
started is listed but flagged **"BUILT AFTER THIS CONSOLE STARTED"**, because
its install prefix is not on the console's `AMENT_PREFIX_PATH` and `ros2 run`
would say "Package not found".

### 5.2 E-STOP is one click with an answer, within 3 seconds

* It goes to `/api/estop`. **Never route it through the generic run path's
  confirm modal or a tab switch.** It runs the same
  `ros2 service call /all/emergency std_srvs/srv/Empty {}` and reports within
  `ESTOP_CONFIRM_S` = 3 s: **confirmed** (exit 0, with the latency) or **NOT
  CONFIRMED** with the reason — a non-zero exit with the output tail, or no
  answer at all.
* Why the deadline exists: `ros2 service call` has no timeout. Against an
  unreachable server — dead, hung, or on another `ROS_DOMAIN_ID` — it waits
  forever and prints nothing, so the operator cannot tell whether the motors
  were cut.
* A pending call is abandoned at `ESTOP_ABANDON_S` = 15 s and killed, so it
  cannot e-stop a server launched later. The banner says not to wait for it: use
  the preflight GUI's `e` key, or cut power.
* No keyboard shortcut, deliberately.
* What a **confirmed** e-stop leaves behind is a firmware state, not a console
  one: the supervisor latches LOCKED, and only a battery power cycle (out and
  in, per drone) clears it. The tile reads `E-STOPPED`.
* The fleet e-stop keeps working even when a drone's per-drone services are
  wedged, because `/all/*` is served from a separate callback group and goes
  through the broadcaster, which never touches a stuck drone's object. That is
  why the wedge documented in CLAUDE.md is rated "high" and not "critical".

### 5.3 YAML writes are text-surgical

`configio.YamlText` rewrites only the lines it touches and **never** calls
`yaml.dump()`: a load/dump round-trip deletes the hard-won comments in these
files — the log-block byte budget, the channel-spacing rule, the Motive `auto`
note, the roster history. The raw editor writes exactly the bytes you typed,
after a `yaml.safe_load()` parse check.

Every write: parse-check → show the diff → timestamped backup into
`console/backups/` (git-ignored) → write. The four editable files are
`crazyflies.yaml`, `motion_capture.yaml`, `server.yaml` and `teleop.yaml`, all
under `src/crazyswarm2/crazyflie/config/`. The pane reminds you that
`crazyflies.yaml` is read **only at server launch**, so restart the server after
a change. Do not introduce a second copy of it: the console edits the one file
the server, the planners and `sync_initial_positions.py` all read.

### 5.4 The rules that keep the console removable

* **Stay decoupled** — [section 1](#1-what-it-is-and-that-it-is-removable).
* **Flight scripts are discovered, never hardcoded.**
  `catalog.discover_scripts()` scans `install/*/share/*/package.xml` for packages
  that depend on `crazyflie_py` and lists their executables; descriptions come
  from each script's module docstring, read with `ast` — **never imported**,
  because importing a flight script can have side effects. Do not re-add
  per-script cards. Give the script a module docstring, or add a `SCRIPT_NOTES`
  entry where a safety detail matters.
* **A gotcha added to CLAUDE.md is only half the job**: add the probe to
  `health.py` too, as a node plus an edge, or as a `SIGNATURES` regex that
  attaches the log line to an existing node.

---

## 6. Honest limits

### 6.1 Operator-paced and interactive runs

**Use the mission window** ([MISSIONS.md](MISSIONS.md)) for an operator-paced
show and for keyboard teleop: the show's prompts become buttons, and the teleop
helper runs on its own pty, driven by an on-screen pad or the keyboard.

* **The bare-Enter gate now works, verified.** Commit `72e54bf` made the
  stdin box send an empty line; it had been read, not tested. On
  2026-10-09 every gate of a paced `escort_show` run in the simulator was
  answered from the mission window's Continue button, which posts `"\n"`
  through the same `/api/input` route — so the backend path is now exercised.
  The same line, now on the Dashboard's output pane, was exercised on
  2026-10-09 against a running launch (logged as `enter`).
* **A pty is still not your terminal**: `sudo` prompts and anything reading
  `/dev/tty` directly are unreliable. `escort_teleop` works because it reads its
  own stdin in raw mode, which on a pty is the console. `teleop_xbox` needs a
  gamepad at `/dev/input/js0` and the stack launched with `teleop:=False` — a
  terminal job.
* **Prefer Abort & land to the output pane's Stop for a show.** Stop escalates
  SIGINT → SIGTERM → SIGKILL, and `abort.py` restores the default handlers once
  its landing begins, so a SIGTERM that arrives during the landing *kills* it. A
  flight script now gets 15 s before that SIGTERM (it was 6 s, against a ~4.5 s
  landing plus a disarm per drone). The mission window's **Abort & land** sends
  one SIGINT and nothing after; Ctrl-C once in a terminal is the same.

### 6.2 Launch options the card does not expose

The launch card passes eight arguments plus an optional `mocap_hostname`. The
remaining `launch.py` arguments have **no field**; use a terminal. The ones that
matter:

* ~~`server:=False`~~ — **now a button** (2026-10-09): Launch → *Start mocap
  only*, the state `sync_initial_positions.py` wants. It leaves the preflight GUI
  off; use a terminal if you want it with no server.
* **`trace_cf:=all`** (or a comma-separated subset) — per-second packet-trace
  summaries from the link layer. It is a *launch argument*, not a `--ros-args`
  flag, and it is read at connect.
* `crazyflies_yaml_file:=`, `motion_capture_yaml_file:=`, `teleop_yaml_file:=`,
  `rviz_config_file:=`.

### 6.3 Health blind spots

* **No box reads `/cfX/kalman_preflight`, the connection statistics, or the
  telemetry watchdog's log lines.** A drone in the "link alive, log data dead"
  stall shows up only indirectly: the per-drone probe times out on
  `/cfN/status --once` and reports "no status message", whose stock fix text
  (scan the address, check logging is enabled) points at the wrong cause. The
  right places to look are the server log for the watchdog's recovery and
  `scripts/link_stall_recorder.py`.
* **A wedged upload looks like five separate failures.** When the server is
  stuck in an upload wait, every drone goes silent at once; the console reports
  five failing drone tiles rather than one stuck server. SIGINT will not stop a
  server in that state — Recovery → "Kill a hung stack" will.
* **Probes are point-in-time.** The absence of a red box is not evidence of
  health — the same trap CLAUDE.md records for the link warnings.

### 6.4 Other limits

* **It is not a safety layer.** The console will happily run a show that should
  not fly. The gates that refuse live in `crazyflie_shows/preflight.py` (the
  supervisor-LOCKED check and the go/no-go), in each planner (`plan_show`,
  `plan_constellation`, `plan_escort`) and in `sync_initial_positions.py`. Run
  the planner after every position sync, from here or from a terminal.
* **Preconditions warn, they do not block** — see the note above section 3's
  tables.
* **Per-drone cards act on the `enabled` drones as of the last catalog build.**
  The catalog rebuilds when `crazyflies.yaml`'s mtime changes, so an edit made
  outside the console is picked up, but a card that was already open is stale
  until the page refreshes.
* **The one-drone Takeoff and Land cards have fixed payloads.** Use the Go-to
  card or a terminal for anything else.
* **No authentication** — [section 2](#2-starting-it).
* **It is only as current as its catalog.** A flag added to a script after its
  card was written is not offered. The argv shown is always what runs.
* **Known drift, not yet fixed:** `catalog.py`'s `SCRIPT_NOTES` hardcodes numbers
  for a few demos (for example the formation demo's "R=1.2 m", "~2.24 m radius"
  and "~58 s"). CLAUDE.md is explicit that this demo's geometry depends on each
  drone's `initial_position` and that no fixed excursion should be quoted for
  it. Those notes should become pointers to the planner output rather than
  figures. Trust `plan_show` / `plan_constellation` / `plan_escort`, not the card.

---

## 7. Extending it

Read [`console/README.md`](../console/README.md) first — it owns the internals.
The short version:

* **A command.** One `action(...)` entry in `mission_console/catalog.py`, with a
  `why` (what it does, when to use it) and a `teaches` (how to read the command
  line). A card without an explanation is just a button, which is the thing this
  console exists not to be. `build(values)` returns an argv list, and that argv
  is both what is shown and what is executed. **Add the row to the table in
  [section 3](#3-gui-action--terminal-command-the-centrepiece) in the same
  commit.**
* **A health check.** A node in `Health.run()`, an edge to its dependency, and a
  probe that sets `status`, `summary`, `detail` and `fix`, and records the exact
  command it ran.
* **An error signature.** One tuple in `SIGNATURES`: the regex, the node it
  belongs to, the severity, and the sentence an operator should read.
* **A flight script.** None of the above. Build the package; it appears on the
  next page load. Restart the console if it was built after the console started.
* **A mission window for a demo.** None of the above either: a
  `missions/<name>.yaml` in the show's package — [MISSIONS.md](MISSIONS.md).
  The console is not edited per demo.

---

**See also:** [`console/README.md`](../console/README.md) (module internals and
the HTTP API) · [`docs/RUNNING.md`](RUNNING.md) (the same operations as terminal
procedure) · [`docs/TROUBLESHOOTING.md`](TROUBLESHOOTING.md) ·
[`docs/MOCAP.md`](MOCAP.md) · `CLAUDE.md` (the gotchas the health diagram
encodes).
