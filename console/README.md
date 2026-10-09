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

Everything lives in `console/`. No code outside this directory depends on it (the top-level README, `CLAUDE.md` and
`docs/` carry prose that points here, and nothing else), nothing in `src/` imports it, and it is not a colcon package — so
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

Health runs on **two cadences**, because the probes have very different
volatility. A **live sweep every 3 s** re-runs only what moves while you watch:
the ROS graph, the `/poses` stream, and each drone's battery, link and
supervisor state. A **full sweep every 30 s** (and on startup, after any action,
after a config write, and on *Re-check*) re-runs everything else — workspace
overlay, Python interpreter, config parse, fleet sanity, the Crazyradio on USB
and the Motive discovery ping. On a single 20 s cadence a drone tile could be
24 s stale, which is a long time to look at a drone and not know it is
e-stopped; the live sweep costs ~0.14 s against ~1.7 s for a full one, because
the expensive probes (the NatNet broadcast ping above all) are exactly the ones
that never change between sweeps. The `↻` stamp in the nav bar reports the live
sweep and its tooltip says when everything else was last verified.

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

**Dashboard** (the default tab) — the cockpit, in five bands, sized to fit the rig
laptop's ~1280x660 viewport (GNOME at 200%) with no scrolling:

* a **fleet row** — one tile per drone plus the radio: the drone's own
  supervisor state in words (`FLYING`, `armed`, `ready to arm`, `E-STOPPED`,
  `FLIPPED`, `CRASHED`, `cannot arm yet`, and combinations such as
  `E-STOPPED + flipped`), battery voltage with a percentage and a
  colour-graded bar, link strength, unicast latency, and a status rail down
  the left edge you can read from across the room. The state is decoded from
  `supervisor_info`; `crazyflie_interfaces/msg/Status.msg` names only seven of
  the bits the firmware packs, so bit 7 (`isCrashed`) is decoded from the
  firmware source and anything above that is reported as an unknown bit rather
  than dropped. The bar's
  *length* is just charge (3.3 V empty → 4.2 V full); its *colour* comes from the
  same thresholds the health probe uses (3.8 V warning, 3.7 V critical), so a
  tile can never look reassuring about a voltage the check calls no-fly;
* a **needs attention** row: every failing or warning check as a chip;
* the **session row** — a stepper over the five steps of a session (before the
  server, fleet positions, bring it up, check, fly) and, beside it, **only the
  current step's buttons**. The stepper follows the rig: nothing up → "before
  the server" (scan, battery, *Start mocap only*); mocap up with no server →
  "fleet positions" (preview / apply the sync, then *Restart with the server*,
  because the server reads `crazyflies.yaml` only when it starts); server up →
  "check" until the `/all/*` services are there, then "fly". A launch is
  refused-with-a-reason while ANY stack is up, mocap-only included — a second
  launch would start a second mocap node on UDP 1511. Click another step to see its
  buttons; that choice holds until the rig changes state. A button that cannot
  work right now is dimmed and the reason is said once at the end of the row.
  Your **pinned** shortcuts sit to the right, and **More** lists every action,
  ranked by how often you have actually run it (the usage log, below), with a ☆
  to pin. This replaced four columns showing all twelve buttons and six
  dropdowns at once, which left the live output five lines tall;
* a **Missions** row: one chip per mission window (below), each opening its own
  browser window;
* a **live console**: everything the session has run down the left, the selected
  one's output streaming on the right, with Stop / Kill / Copy and a stdin line
  (see "No Processes tab" below).

Nothing is defined twice: each button runs the catalog action, its tooltip is the
real argv, and its `⋯` opens the full card in Control. Every tile, chip and pill
is a link to the check behind it.

The header carries one **verdict** — ALL SYSTEMS GO / ATTENTION / FAULT — with
the headline behind it, and the nav bar carries the live telemetry: server,
mocap, `/poses` (with a sparkline of the last ~40 samples, so a dropout is
visible as a dive rather than a number that briefly changed), fleet size, ROS
domain, and how long ago the probes last ran.

**Build status of the shows** (`mission_console/builds.py`). Every
`crazyflie_py`-dependent package under `src/` is compared with `install/`:
a package never built, a script declared in `setup.cfg` with no generated
shim, a new module file with no installed symlink, or a package built after
the console started (not on its `AMENT_PREFIX_PATH`). The symlink install
makes *edits* live and hides exactly these. It feeds the **Show packages
built** box in System health (so it lands in "needs attention"), a *not
built* badge on mission chips, a Build button in the mission window (the
catalog's own `./scripts/build.sh <pkg>`), and the picker's list of shows with
no mission view. A new module blocks only the scripts that import it (their
same-package import closure, read with `ast`).

**Mission windows** (`/mission?m=<pkg>/<name>`) — a dashboard per demo,
defined by the demo: a `missions/<name>.yaml` in the show's package says what to
run, what the operator chooses before starting, which helpers go with it
(keyboard teleop), how to recognise a prompt that waits for the operator, and
what to pull out of the output as status. The window renders that with a live 3D
view (three.js, fed straight from foxglove_bridge on :8765, read-only), the
prompts as buttons, the teleop as a key pad, and an **Abort & land** that sends
exactly one SIGINT so the show's own `abort.py` landing is never chased by a
SIGTERM. A new demo is a new YAML file, not a console change. Spec format,
safety contract and what was verified: [`docs/MISSIONS.md`](../docs/MISSIONS.md).
`/mission` alone lists them; a spec that does not validate is listed with its
error. The built-in **Any flight script** window runs any discovered script
with the generic `>>>` gate.

**Control** — the reference: one verbose card per action, grouped Launch,
Preflight, Fleet position, Flight, Commands (service calls), Parameters,
Recovery. Actions declare whether they need the server running or stopped, and
say so rather than silently failing. Anything that moves a drone asks for
confirmation and shows the command first.

**Flight scripts are discovered, not listed.** Every package in the workspace
that depends on `crazyflie_py` is scanned for executables — `crazyflie_examples`,
`crazyflie_shows`, and whatever you build next — and offered in one *Run a flight
script* card, grouped by package, each with its own module docstring as the
description (read with `ast`, never imported). Scripts that never command a
drone (`plan_*`, `*.sh`, `color_led`, `set_param`) go to *Run a ground check*
instead, without the "area clear?" prompt. Build a new show package and it
appears on the next page load; if it was built after the console started, it is
flagged until you restart the console from a sourced shell.

**Command palette** (`Ctrl-K`, or `/`) — one search box over every catalog
action, every health check, every config file, every process and every tab.
`Enter` runs the highlighted action with its remembered parameters, through the
same confirm step as the buttons — so it can never quietly fly a drone;
`Shift-Enter` opens its full card in Control instead. Action ids are search keys,
so "fly" finds `fly.script` even though no word of its label says it.

**Keyboard** (`?` shows the list) — `1`…`5` switch tab, `r` re-runs every probe,
`Esc` closes whatever is open. E-STOP deliberately has **no** shortcut: a stray
keypress must never cut the motors.

**E-STOP** (header, every tab) — one click, **no confirmation, no tab switch**,
the same contract as the preflight GUI. It runs
`ros2 service call /all/emergency std_srvs/srv/Empty {}` and a banner reports
what happened within 3 s: confirmed (with the latency), or **NOT CONFIRMED** with
the reason. That deadline matters: `ros2 service call` has no timeout of its own,
so an unreachable server — dead, hung, or on a different `ROS_DOMAIN_ID` than the
console — used to leave the call waiting forever with no output at all. A pending
call keeps retrying for 15 s in case discovery is just slow, then is abandoned so
it cannot e-stop a server launched later.

**Restart the console after updating it.** The page is re-read from disk on every
load but the backend is not, so a reloaded page can call routes the running
process does not have — they answer `not found`. The page now detects this and
shows an *out of date* banner (and warns on load when the code changed since the
console started); the E-STOP falls back to a route every version has, so it still
fires. Just stop `./console/run.sh` and start it again.

**Config** — edit `crazyflies.yaml` as a table (enable/disable, URI, position,
type, add/remove a drone), or any of the four config files as raw text. Writes
are **parse-checked**, show you a **diff** before they land, take a timestamped
**backup** into `console/backups/`, and **preserve comments** — structured edits
rewrite only the lines they touch, never a `yaml.dump()` round-trip. The pane
reminds you that `crazyflies.yaml` is read only at launch.

**No Processes tab** (folded into the Dashboard 2026-10-09 — the two showed the
same thing). The Dashboard's live console is the process view: every command
the console started down the left (the tab badge counts the ones still
running), the selected one's output on the right — appended as it arrives, so
scrolling back is not undone; children run on a pty, so output is line-buffered
as in a terminal — with Stop (SIGINT to the whole process group, escalating; a
flight script gets 15 s before the SIGTERM), Kill (SIGKILL), Copy, follow, and
a stdin line for interactive helpers (`scripts/led.sh`, a paced show's bare
Enter). An unanswered prompt with no newline (`input('...: ')`) shows at the
bottom of the pane.

**Command log** — the session as a shell script, plus **Usage across
sessions**: what has been run, how often, started from where (dashboard, card,
palette, pin, More, mission window), which tabs and missions are used.

**The usage log** (`mission_console/usage.py`) is how the dashboard's layout
stops being a guess. Every run, stop, e-stop, config write, scan, tab, palette
use and mission open/start is appended to `console/usage/usage.jsonl`
(git-ignored, like `backups/`); pins go to `console/usage/dashboard.json`.
stdin is recorded only as its shape (`enter`, `q`, `text`) because the stdin box
is also where a sudo password goes, and teleop keystrokes are not recorded one by
one. It is a plain file on purpose: read it from a shell, or ask Claude to,
before rearranging anything. `MISSION_CONSOLE_USAGE_DIR` redirects it — set it
for a test console so test clicks never count as the operator's.

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

*Add a mission window:* **do not touch the console.** Write
`missions/<name>.yaml` in the show's package ([`docs/MISSIONS.md`](../docs/MISSIONS.md));
a widget the stock set lacks goes in a `view:` module next to the spec. Change
the console only for something every mission needs — a new stock widget in
`static/mission/app.mjs` (register it in `STOCK`, and add its name to
`missions.WIDGETS` so specs validate), a new marker type in `scene.mjs`, a new
spec field in `missions.normalise()`.

*System API* (`server.py`): `GET|POST /api/settings` (the single store;
POST `{values: {key: value|null}}`, broadcast as an SSE `settings` event),
`GET /api/system` (the census), `POST /api/signal {pid, mode: int|kill}` (only
a pid the census lists as an OUTSIDE rig process), `GET /api/journal`; SSE
`fleet` events carry each journal entry. Health facts gain `system`,
`server_any`, `mocap_any`, `stack_any` (machine-wide) beside the per-domain
`server_running` / `mocap_running`. See docs/CONSOLE.md 5.0.

*Mission API* (`server.py`): `GET /api/missions` (cards), `GET /api/mission?id=`
(spec + its processes), `POST /api/mission/preview` and `/api/mission/start`
(`{id, role: main|<helper>, values, sim}` -> argv / a process), `GET /api/arena`
(arena.yaml + parking marks), `GET /mission-assets/<pkg>/<name>/<file>` (a
custom view module, path-checked). `POST /api/stop` takes `mode: stop | kill |
interrupt` (`interrupt` = one SIGINT, no escalation). `GET|POST /api/usage` and
`/api/prefs` serve the usage log and pins. Every process the window starts uses
the same pty/process manager as the rest of the console, so it shows up in the
Dashboard's activity list and the Command log.

Probes deliberately go through the `ros2` CLI rather than `rclpy`: the CLI
daemon keeps the ROS graph warm, while a fresh `rclpy` process on this rig can
stall in DDS discovery and never return (the same reason `scripts/led.sh` uses
the CLI). It also means every probe is a command you can run yourself. The one
exception is the mission window's 3D view, which reads foxglove_bridge directly
from the browser: a 50 Hz stream is not a command, and the bridge is already
part of the launch.

## Layout

```
console/
  run.sh                     entry point (sources ROS + overlay, de-condas, serves)
  stop_stack.sh              graceful stack stopper (SIGINT -> SIGTERM -> SIGKILL, then verifies UDP 1511 is free)
  backups/                   timestamped copies of every config file it writes
  mission_console/
    __main__.py              CLI, http server lifecycle, signal handling
    server.py                JSON API + SSE event stream
    catalog.py               the command catalog (every button)
    health.py                the health graph probes and error signatures
    configio.py              YAML read, validation, comment-preserving writes
    procs.py                 process manager (pty, process groups, ring buffers,
                             partial-prompt events, the SIGINT-only interrupt)
    missions.py              mission-spec discovery, validation and argv
    builds.py                is every show in src/ built? (no imports, file checks only)
    settings.py              the ONE store for every dropdown/field (console/usage/settings.json)
    system.py                census of rig processes on the machine: any domain, any origin
    journal.py               fleet-command journal from /rosout (ros2 topic echo --csv)
    usage.py                 the usage log and dashboard prefs
    static/                  index.html, app.js, style.css
      mission/               the mission window: index.html app.mjs ros.mjs scene.mjs plan2d.mjs mission.css
      vendor/                Preact+htm, three.js (vendored: the rig network is offline)
  tests/                     python3 -m unittest discover -s tests
  usage/                     usage.jsonl + dashboard.json (git-ignored)
```

Flight scripts are discovered, not hard-coded: `catalog.discover_scripts()` scans
every `crazyflie_py`-dependent package under `install/` (so the `crazyflie_shows`
scripts appear automatically, with an escort-specific stdin note in `catalog.py`).
The operator-facing GUI-action-to-command table lives in
[../docs/CONSOLE.md](../docs/CONSOLE.md); this file is for editing the console.

Tests (stdlib `unittest`, no ROS needed — the spec checks read the installed
packages, so source the workspace first):

```bash
cd console && python3 -m unittest discover -s tests -v
```

`?live=0` on the URL opens the page without the live event stream (a static
snapshot, useful for a screenshot — and required for a headless-browser capture,
which otherwise waits forever on the open stream). `?node=<id>` opens with that
health box selected, which is what the dashboard's links use.

There is no UI-scale control any more: use the browser's own zoom (Ctrl +/−),
which Chrome and Firefox remember per site.
