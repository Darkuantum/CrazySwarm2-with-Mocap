# Mission console

A browser GUI for running this rig: the launch, the preflight checks, the flight
commands, the YAML config, and a health diagram that says **where** the stack is
broken instead of making you read a launch log.

```bash
./console/run.sh                 # opens http://localhost:8077
./console/run.sh --port 9000
./console/run.sh --no-browser
./console/run.sh --host 0.0.0.0  # reachable from the lab network -- see "Access" below
```

`run.sh` sources ROS and this workspace for you (and strips conda), so you can
start it from any shell.

---

## It is a module, not a fork

Everything lives in `console/`. **No file outside this directory was changed to
add it**, nothing in `src/` imports it, and it is not a colcon package — so
`rm -rf console/` removes the feature completely and the workspace keeps working
exactly as documented in the top-level README. It also needs no rebuild: it is
plain Python 3 + PyYAML (already a dependency) driving the `ros2` CLI.

The console never talks to the drones itself. It runs the same commands you
would type, as child processes of a normally-sourced shell.

---

## Why the commands are always visible

The point is not to hide `ros2` from you but to stop you typing it from memory.
So:

* every button shows the **exact command line** it will run, live, as you change
  its parameters — and that string comes from the backend that will execute it,
  so it cannot drift out of sync with what actually runs;
* every card has a **"How to read this command"** note explaining the syntax
  (`name:=value` is launch syntax, `--once` exits after one message, a service
  call's last argument is YAML for the request fields, ...);
* every health check shows **the command it used** and that command's output;
* the **Command log** tab keeps every command of the session and exports it as a
  runnable shell script (`Download as a shell script`).

Copy any of it into a sourced terminal and it does the same thing.

---

## The tabs

**System health** — the flow diagram. It follows the real data path:

```
workspace -> config -> radio + Motive/UDP1511 -> mocap node -> /poses
                                              -> crazyflie_server -> /all/* -> each drone
```

Each box is an independent probe. Click one for what was measured, the command
behind it, and the fix. The failures this rig actually hits are encoded as
boxes, so they name themselves instead of hiding:

| Box | Catches |
|-----|---------|
| Workspace overlay / Mocap driver build | the apt `motion_capture_tracking` shadowing the vendored one (hard-coded IP 141.23.110.162 → silent `/poses`) |
| Python interpreter | a conda python shadowing `/usr/bin/python3` |
| Fleet sanity | duplicate addresses, drones under 1 m apart, unknown types, two dongles under 2 channels apart, an over-budget firmware log block |
| Motive / NatNet | Motive not answering the discovery ping (the same ping `launch.py` uses) |
| UDP 1511 | the leftover socket that starves the mocap node into a silent hang |
| Drones answer radio | a dead drone or a datarate mismatch — **before** it hangs the server |
| /all/* services | the server alive but blocked forever mid-connect, the classic silent hang |
| each drone | connected?, battery, rssi, unicast latency |

Boxes downstream of a hard failure go grey ("blocked upstream") so you look at
the cause, not the symptoms. Anything the console has run is also scanned for
known error signatures, and a match is attached to the box it belongs to.

Boxes that do not apply to the current run (radio and mocap during a
`backend:=sim` launch) show as "not applicable" rather than red.

**Control** — the commands, grouped: Launch, Preflight, Fleet position, Flight,
Commands (service calls), Parameters, Recovery. Actions declare whether they
need the server running or stopped, and say so rather than silently failing.
Anything that moves a drone asks for confirmation and shows the command first.
The E-STOP button in the header calls `/all/emergency` from any tab.

**Config** — edit `crazyflies.yaml` as a table (enable/disable, URI, position,
type, add/remove a drone), or any of the four config files as raw text. Writes
are **parse-checked**, show you a **diff** before they land, take a timestamped
**backup** into `console/backups/`, and **preserve comments** — structured edits
rewrite only the lines they touch, never a `yaml.dump()` round-trip. The pane
reminds you that `crazyflies.yaml` is read only at launch.

**Processes** — every command the console started, with live output (it runs
children on a pty, so output is line-buffered as in a terminal), Stop (SIGINT to
the whole process group) and Kill (SIGKILL), plus a stdin box for interactive
helpers such as `scripts/led.sh`.

**Command log** — the session as a shell script.

---

## Access

It binds `127.0.0.1` by default. `--host 0.0.0.0` makes it reachable from the
lab network — convenient from a laptop, but there is **no authentication**:
anyone who can reach the port can arm and fly the drones. Use it only on a
trusted network, and prefer an SSH tunnel otherwise:

```bash
ssh -L 8077:localhost:8077 <rig-host>
```

Closing the console (Ctrl-C) sends SIGINT to anything long-running it started,
the same as closing the terminal you had launched them from.

---

## Extending it

*Add a command:* one `action(...)` entry in `mission_console/catalog.py`. Give
it a `why` (when to use it) and a `teaches` (how to read the command) — a card
with no explanation is just a button, which is the thing this console is trying
not to be. `build` receives the parameter values and returns an argv list.

*Add a health check:* add a node id/label/column/row to the list in
`Health.run()` in `mission_console/health.py`, an edge to its dependency, and a
probe that sets `status`, `summary`, `detail`, `fix`, and appends the command it
ran to `commands`.

*Add an error signature:* one tuple in `SIGNATURES` (regex, node id, level,
title, explanation, fix) makes any matching line in any process output surface
on that box.

Probes deliberately go through the `ros2` CLI rather than `rclpy`: the CLI
daemon keeps the ROS graph warm, while a fresh `rclpy` process on this rig can
stall in DDS discovery and never return (the same reason `scripts/led.sh` uses
the CLI). It also means every probe is a command you can run yourself.

## Layout

```
console/
  run.sh                     entry point (sources ROS + overlay, de-condas, serves)
  backups/                   timestamped copies of every config file it writes
  mission_console/
    __main__.py              CLI, http server lifecycle, signal handling
    server.py                JSON API + SSE event stream
    catalog.py               the command catalog (every button)
    health.py                the health graph probes and error signatures
    configio.py              YAML read, validation, comment-preserving writes
    procs.py                 process manager (pty, process groups, ring buffers)
    static/                  index.html, app.js, style.css
```

`?live=0` on the URL opens the page without the live event stream (a static
snapshot, useful for a screenshot).
