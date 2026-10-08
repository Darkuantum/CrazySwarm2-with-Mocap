# SERVER-CHANGES — what this fork changed in the server, versus upstream

**Audience:** a maintainer re-applying these changes onto a newer upstream, or deciding which of
them to offer back.

Every claim below is tagged **[diff]** (read in `git diff upstream/main HEAD`, 2026-10-08),
**[CLAUDE.md]** (documented there with its own evidence, dates and disproved theories) or
**[inferred]** (reasoning from the code, not measured).

## Regenerate this diff before trusting the document

```bash
cd /home/jeremy/CrazySwarm2-with-Mocap
git fetch upstream                      # fetch-only remote; its push URL is deliberately bogus
git merge-base upstream/main HEAD       # 9ed4bae as of 2026-10-08 — no merge/rebase since

# exact per-file added/removed (NOT --stat, see the note under the table)
git diff --numstat upstream/main HEAD -- \
  src/crazyswarm2/crazyflie/src/crazyflie_server.cpp \
  src/crazyswarm2/crazyflie/config/server.yaml \
  src/crazyswarm2/crazyflie/launch/launch.py \
  src/crazyswarm2/crazyflie/deps/

# the server diff itself
git diff upstream/main HEAD -- src/crazyswarm2/crazyflie/src/crazyflie_server.cpp
```

If those numbers no longer match the ones quoted here, the numbers here are stale — the command
is the ground truth, this prose is not.

## Summary

| # | group | lines | why | portable? |
|---|---|---|---|---|
| 1 | Telemetry watchdog (detect + 2-tier recovery of dead log blocks) | ~170 fn + ~60 builder refactor, in `crazyflie_server.cpp` | a drone silently deletes its own log blocks after a 1 s receive gap; only a server restart used to recover it | mechanism yes (offer default-off), tuning rig-specific |
| 2 | Bounded trajectory upload + `bad_trajectories_` refusal | within `crazyflie_server.cpp` + `Crazyflie::uploadTrajectory` | an upload could wedge the whole server permanently, or report success for a trajectory with a hole in it | **yes — strongest candidate** |
| 3 | Airborne gate (`may_be_airborne` / `commanded_flight_` / mocap z) | within `crazyflie_server.cpp` | never run `logReset` on a drone that might be flying | only with mocap |
| 4 | Keep-alive timer (`keepalive_frequency`) | small | guaranteed per-drone unicast floor under the firmware's 1 s timeout | yes, but it is insurance, not a fix |
| 5 | Packet tracing (`trace_cf`) | server + 4 link-cpp files | the diagnostic that localised the stall to the drone | yes, cheap when off |
| 7 | `server.yaml` delta | 54 / 3 | see table in §7 | partly (two keys are rig-specific) |
| 8 | `launch.py` delta (`trace_cf`, `server:=False`) | 26 / 3 | mocap-up/server-down state that `sync_initial_positions.py` needs | yes |
| 9 | Library changes the server depends on | 580 changed lines across `crazyflie_cpp` | identity-matched replies, bounded waits, `abandon()`, trace hooks | yes — section 9 contains the other strongest candidates |

**Exact file totals [diff]:**

| file | added | removed | note |
|---|---|---|---|
| `crazyflie/src/crazyflie_server.cpp` | 445 | 22 | 1760 → 2183 lines (net +423) |
| `crazyflie/config/server.yaml` | 54 | 3 | |
| `crazyflie/launch/launch.py` | 26 | 3 | |
| `crazyflie/deps/.../crazyflie_cpp/**` | 512 | 68 | `Crazyflie.h` 266/68, `Crazyflie.cpp` 212/9, `crtp.h` 51/0, link-cpp 51/0 across 4 files |

Note on arithmetic: the "+467 / +57 / +29" figures quoted elsewhere in the repo are
`git diff --stat` sums (added **plus** removed: 445+22, 54+3, 26+3), not lines added. Use
`--numstat`.

## 0. Read this first: what "upstream" is, and the corrections it forces

- `upstream` = `AI-DA-STC/CrazySwarm2-with-Mocap`, tip `9ed4bae` (Thu 10 Sep 2026), which is also
  `git merge-base upstream/main HEAD`. The fork has not been rebased or merged since that
  date. **[diff]**
- Upstream AI-DA-STC is itself a derivative of `IMRCLab/crazyswarm2`. Anything that happened
  *before* that fork point is invisible to a diff against `upstream/main`.
- **The server cannot be re-applied from `crazyflie_server.cpp` alone.** It calls API that exists
  only in the vendored libraries (§9). Porting the server file without them will not compile.

### Correction to CLAUDE.md

Several things CLAUDE.md lists as this fork's `crazyflie_server.cpp` / `launch.py` changes are
**already present at `upstream/main`** and do not appear in the diff **[diff]**:

- `add_on_set_parameters_callback` instead of `ParameterEventHandler` (upstream
  `crazyflie_server.cpp:1311` carries the "NOTE: upstream used a ParameterEventHandler" comment;
  the callback is at 1319).
- Color LED deck green on connect, off on clean disconnect (upstream lines 381–395, 580–606).
- `query_all_values_on_connect: True` in `server.yaml` (upstream line 14; this fork only
  lowercased `True` → `true`, cosmetic).
- The foxglove_bridge and preflight-GUI nodes, and the `rviz` / `preflight` / `foxglove` launch
  arguments in `launch.py` (upstream lines 252–255, 286, 307–319).

They are still true *of the rig* — they are just not part of what must be re-applied on top of a
newer AI-DA-STC. Against IMRCLab they probably **would** be fork changes (not verified, see
"Known gaps"). CLAUDE.md should say "already in AI-DA-STC" rather than "this fork added".

## 1. Telemetry watchdog: detect and recover from "link alive, log data dead"

**Where:** `CrazyflieROS::check_telemetry_watchdog()`, `restart_log_blocks()` (tier 1),
`rebuild_log_blocks()` (tier 2), `add_log_builder()`; `on_link_statistics_timer()` now calls the
watchdog first; `last_log_rx_` stamped in all five `on_logging_*` callbacks; `last_latency_rx_`
stamped in `on_latency`. Config: `telemetry_watchdog_s` (`server.yaml`, `5.0`).
**Trap:** the C++ fallback when the key is absent is `0.0` = **off**, so a yaml that lacks the key
silently disables recovery.

Roughly 170 added lines for the three functions plus ~60 for the builder refactor and state
(approximate — see "Known gaps"; only the file totals above are exact).

**What upstream does:** each log block is built inline —
`log_block_pose_.reset(new LogBlock<logPose>(&cf_, {...}, cb)); ...->start(period)`. Nothing
watches for silence and nothing can re-create a block. A mute drone stays mute until the server is
restarted. **[diff]**

**What the fork does:**

1. Each of the five block kinds (pose, scan, odom, status, generic/custom) is registered through
   `add_log_builder(lambda)`, which runs the lambda once (identical behaviour at connect) and
   stores it in `log_block_builders_`. **The closure is the only record of how to make that block
   again.** Periods are kept in `period_pose_`, `period_scan_`, `period_odom_`, `period_status_`,
   `periods_generic_`. `publishers_generic_` is a `std::list` so the `user_data` pointer captured
   by the generic builder stays valid. **[diff]**
2. A drone is declared stalled when `quiet >= telemetry_watchdog_s` (no log callback) **and** its
   latency echo is current (`since_latency <= 2.0 s`; otherwise it is an ordinary dead link, not
   ours to fix) **and** `may_be_airborne()` is false (§3). Recovery sets `recovering_` and runs
   two tiers:
   - **Tier 1** `restart_log_blocks()`: `stop()` then `start(period, RECOVERY_TIMEOUT_MS)` per
     block. Cheap; works only if the blocks still exist on the aircraft.
   - **Tier 2** `rebuild_log_blocks()` (when tier 1 throws): `cf_.logReset(RECOVERY_TIMEOUT_MS, 2)`
     **first, as a probe** (false → throws, nothing destroyed), then `abandon()` and free every
     host-side block, clear `periods_generic_`, set `recovery_timeout_ms_`, and re-run every
     builder, re-checking `may_be_airborne()` between builders.
   - A rebuild that fails half-way sets `rebuild_pending_`; the retry is driven by that flag at the
     same cadence, because a half-built set still delivers some data so `quiet` never grows. On a
     retry tier 1 is **skipped** — it would "succeed" on the surviving blocks and never recreate
     the missing ones.
3. `RECOVERY_TIMEOUT_MS = 1000`, applied ×2 tries. It was 300 ms and was raised after a real
   `start` answered later than that **[CLAUDE.md]**. `recovery_timeout_ms_` is 0 at connect
   (unbounded, deliberately) and non-zero only for the duration of a rebuild; every exit path
   resets it to 0.

**Why — the failure, with evidence [CLAUDE.md]:** from 2026-10-02, five drones on one Crazyradio
2.0 would randomly publish **exactly 0.00 Hz** of `/cfX/pose`, `/cfX/status` and
`/cfX/kalman_preflight` while still answering latency pings and accepting `ros2 param set`. Packet
trace 2026-10-05: `log=0 null=~200 other=1` per second — zero log packets ever leave the drone.
Root cause read in the firmware (`~/crazyflie-firmware`, tag 2025.02): `log.c logRunBlock()` runs
`logReset(); crtpReset();` when `crtpIsConnected()` is false, and `radiolink.c` defines that as
"the drone received a packet within `M2T(1000)`". **One second of the drone receiving nothing makes
it delete all its log blocks, and it never tells the host.** Tier 1 coming back with
`Could not start log block!` at a real cf5 stall — a *response*, not a timeout — is the direct
proof that the blocks were gone.

**Verified on hardware 2026-10-05:** cf5 stalled twice in a 20-minute run; each time tier 1 was
refused, tier 2 recreated the blocks, and all three topics were back **~1.0 s** after the stall was
declared — no server restart, no power cycle. What causes the ≥1 s receive gap in the first place
is **still OPEN** (CLAUDE.md "STILL OPEN"; the live unmeasured candidate is the broadcast-priority
skip in `CrazyradioThread.cpp`).

**Ruled out by measurement — do not re-chase [CLAUDE.md]:** telemetry rates, the console, CPU and
graphics load, pty back-pressure, DDS config, the dongle, drone firmware, leftover log blocks, the
radio loop skipping connections, and every host-side explanation (nothing arrives to drop).

**If reverted:** the stall returns and only a server restart clears it — mid-show.
**Do not reorder the probe-before-destroy in `rebuild_log_blocks()`:** the original order freed all
host blocks before an unanswered `logReset`, leaving a drone with no dispatch at all while the
aircraft may still have held live blocks.

**Portable?** The mechanism is generic to any Crazyswarm2 server on a saturated single dongle, so
it is a reasonable upstream offer. The tuning is rig-specific (5 s, 1000 ms ×2, `z <= 0.10 m`).
Split the PR: builder refactor first (behaviour-neutral), watchdog second, default `0.0`.
A rig **without mocap** cannot use the airborne gate as written and needs a different interlock
(see §3) before enabling this at all.

**Known wart [inferred]:** a tier-2 rebuild runs on `callback_group_cf_srv`, which is fleet-wide
and mutually exclusive, so a worst-case rebuild freezes all five drones' services and telemetry for
up to ~14 s. Tracked as OPEN in CLAUDE.md; not changed on purpose, because `processAllPackets` and
`waitForResponse` pop the same per-connection queue and the `Crazyflie` object has no mutex.

## 2. Bounded trajectory upload, `bad_trajectories_`, and refusing to fly a failed upload

**Where, server:** the `upload_trajectory` handler (try/catch + `response->success`/`message`),
`trajectory_is_bad()`, `mark_trajectory_bad()`, `bad_trajectories_` + `bad_trajectories_mutex_`,
and the REFUSE blocks in both the per-drone `start_trajectory` and the broadcast
`CrazyflieServer::start_trajectory`. **Where, library:** `Crazyflie::uploadTrajectory` (§9).

**What upstream does:** `cf_.uploadTrajectory(...)` with an unbounded `waitForResponse`, no
try/catch, no success reporting; `start_trajectory` starts whatever id it is handed.

**What the fork does:**

- The upload throws after bounded retries (500 ms × 2 tries per wait, chunk resent up to 3 times,
  10 s per chunk, 90 s per call). The handler catches, logs FATAL, sets `success=false`, and
  records the id via `mark_trajectory_bad()`. A later clean upload of the same id clears it.
- `start_trajectory` (unicast) returns early with a FATAL log for a bad id. The **broadcast**
  handler scans every drone and **refuses for everyone** if any drone has that id marked bad — the
  formation holds position instead of one drone flying garbage next to four flying the figure.
- The set is mutex-guarded: written on `callback_group_cf_srv`, read from the broadcast handler on
  `callback_group_all_srv_`. Genuinely concurrent; an unguarded `std::set` is UB.

**Why — the failure, with evidence [CLAUDE.md]:** 2026-10-05 mid-show, all five drones stopped
publishing during a trajectory upload and did **not** recover when the show was stopped, because
the *server* was stuck. The chain: the upload saturates the radio and starves other connections
(measured `[cf5] Low unicast receive rate (0.10 < 0.90). Sent: 2807. Received: 289`) → the starved
drone crosses the firmware's 1 s activity timeout and `crtpReset()` flushes **the memory-write
response the server is waiting for** → the unbounded wait never returns → the mutually exclusive
`callback_group_cf_srv` therefore disables `land`, `takeoff`, `arm`, `emergency` and the 1 ms
`spin_once` that dispatches all log data. Signature in the log: `[cfX] upload_trajectory(...)` with
nothing after it. SIGINT cannot stop a server in that state; it needs SIGKILL, and `ros2 launch`
leaves the node behind.

Second incident the same day: the show flew cf5 through four trajectories it never received
(`Sent: 1487. Received: 0`) and it flew whatever bytes were already in that memory slot until it
was e-stopped. That is why the **broadcast** refusal exists — shows start trajectories over the
broadcast path, which never touches a `CrazyflieROS`.

**If reverted:** one starved drone can wedge the whole server permanently, and a show can fly a
partially written trajectory. `piecewise_eval` applies whatever bytes are there with **no
validation, no clamp and no finiteness check**, and the supervisor only reacts to TUMBLED and
battery **[CLAUDE.md]**.
**Trap: do not "simplify away" the `bad_trajectories_` refusal**, and do not drop the mutex.

**Portable?** Yes — high value, low controversy, rig-independent. Note that
`UploadTrajectory.srv` already carries `success`/`message` (and `crazyflie_py` raises on it);
`StartTrajectory.srv` has no response fields, which is exactly why a server-side set is needed on
the broadcast path. Companion show-side check: commit `0d9ca40`.

## 3. The airborne gate: `may_be_airborne()` / `commanded_flight_` / mocap z

**Where:** `may_be_airborne()`, `note_commanded_flight()`, `note_mocap_z()`,
`CrazyflieServer::note_commanded_flight_all()`; calls in per-drone `takeoff`, `go_to`,
`start_trajectory`, in the broadcast `takeoff`, `go_to`, `start_trajectory`, and in the `/poses`
handler (`note_mocap_z(pose.pose.position.z)` for every pose whose name is a known drone). The
members `commanded_flight_`, `last_mocap_z_valid_`, `last_mocap_z_` are `std::atomic`.

**What upstream does:** nothing — there is no recovery to gate.

**What the fork does:** the watchdog may touch a drone's connection only if mocap positively shows
`z <= 0.10 m` **and** `commanded_flight_` is clear. **Unknown altitude counts as airborne.** Mocap
is the only signal that clears the latch; commands set it. Mocap rather than `/cfX/status` because
telemetry is dead exactly when the status topic would be asked, and the latch covers `/all/*`
because shows take off with a broadcast that no per-drone handler sees.

**Evidence and history [CLAUDE.md + diff]:** `commanded_flight_` was declared false, set false and
read once but **never set true** until 2026-10-06, so the gate was the mocap test alone while a
comment implied more. The gate was also evaluated once, although the bounded worst case to the last
builder is several seconds — hence the re-check between builders. The three fields were plain
`bool`/`float` written on `callback_group_mocap_` and read on `callback_group_cf_srv`: a data race
on the single guard deciding whether to touch a possibly airborne drone. They are now atomic.

**If reverted:** the watchdog could run `logReset` on a flying drone. Whether that is actually
harmful in flight has not been measured here **[inferred]** — the guard is fail-safe by design;
keep it.

**Portable?** Only with mocap, and only together with the watchdog, behind the same parameter. A
rig without mocap needs a different "definitely on the ground" signal before enabling §1 — e.g. the
supervisor's own `CAN_BE_ARMED` / flying bits from `/cfX/status`, which on this rig is unusable
precisely because that topic is the one that went silent.

**Gap, partly resolved 2026-10-08 [diff + source]:** the `/all/*` latch covers `takeoff`, `go_to`
and `start_trajectory`. There is deliberately none in `land`/`emergency` (they must not latch). A
read of `crazyflie_server.cpp` confirms `notify_setpoints_stop` and both `cmd_full_state_changed`
handlers do NOT call `note_commanded_flight()`. The escort demo streams `cmdFullState`, but
`escort_show.py` (line ~708) begins with `cf.takeoff(...)`, which does latch, and nothing clears
the latch except mocap z > 0.10 m. So the gate holds for the escort as written. A show that streamed
`cmdFullState` WITHOUT a prior takeoff/go_to/start_trajectory call would not latch; the airborne gate
would then rest on mocap z alone.

## 4. Keep-alive

**Where:** `keepalive_timer_`, `keepalive_hz_`; the `keepalive_frequency` parameter is declared in
the `CrazyflieServer` constructor (default 4.0) and mirrored in `server.yaml`. The timer calls
`cf_.triggerLatencyMeasurement()` on **`callback_group_cf_cmd`** — not `_cf_srv` — so a tier-2
rebuild cannot silence it.

**Upstream:** no such timer.

**Why [CLAUDE.md]:** a guaranteed per-drone unicast floor (4/s) under the firmware's 1000 ms
activity timeout. **Honest status: insurance, not a fix.** A healthy drone on this rig already
receives ~168 unicast/s (`num_rx_unicast`, measured 2026-10-05) because the link library's own
auto-pings refresh the same tick, and a connection starved for a full second starves the keep-alive
too — both go through the same radio thread. An earlier guess that the 1 Hz warning ping sat on the
threshold was measured and withdrawn.

**If reverted:** probably nothing visible today. It protects against auto-ping behaviour changing.

**Portable?** Trivially, but upstream will ask for evidence that it helps and there is none beyond
the argument.

## 5. Packet tracing (`trace_cf`)

**Where, server:** the `debug.trace_cf` parameter, parsed in the `CrazyflieROS` constructor
(`"all"`, `"*"`, or comma-separated names with space trimming) → `cf_.setTrace(true)` plus a WARN
line. **launch.py:** `LaunchConfiguration('trace_cf')` folded into
`server_params[1]['debug.trace_cf']`, with `DeclareLaunchArgument('trace_cf', default_value='')`.
**Library:** `Connection::setTrace`, the `ConnectionImpl::trace_*` counters, a block in
`CrazyradioThread.cpp` (a relaxed atomic load per ack when off), and `Crazyflie::m_trace*` in
`processPacket`.

**Why [CLAUDE.md]:** this is what localised the stall to the drone. The link layer reads
`log=0 null=~200`, the host layer agrees (`log=0 dispatched=0`). One summary per second per drone,
because per-packet printing from the radio thread would be ~825 lines/s across five drones and
would perturb the very timing under measurement; per-packet lines appear only for anomalies (a
valid-but-unregistered log packet, a port-5 packet failing `valid()`).

**Traps:** it is a **launch argument**, not `--ros-args` (`ros2 launch` has no such flag), and it is
read **at connect only**, because this rig's `/parameter_events` never loop back.
**Stale comment [diff]:** the code comment above the parser still shows
`--ros-args -p debug.trace_cf:=cf1`, which does not work through `ros2 launch` — trust `launch.py`.
The same hunk also reads oddly: the `telemetry_watchdog_s` declaration sits in the middle of the
trace section. Cosmetic, but a reviewer will notice.

**If reverted:** nothing breaks; you lose the diagnostic.
**Portable?** The library hooks are generic and cheap when off. Offer it.

## 6. Why three CLAUDE.md "fork changes" are not in this diff

`add_on_set_parameters_callback`, LED-green-on-connect and `query_all_values_on_connect` are
already in `upstream/main` (§0). **If you rebase onto something that is not AI-DA-STC (e.g.
IMRCLab), re-apply them**: the parameter-callback change is required because on this rig a node's
own `/parameter_events` are never delivered back to itself, so upstream's `ParameterEventHandler`
never fires and `ros2 param set` silently never reaches the drones **[CLAUDE.md]**. The LED uses
`colorLedBot.wrgb8888` (`0xWWRRGGBB`, green `0x0000FF00`) plus an off-command on clean disconnect;
the "red while a script flies" half lives in `crazyswarm_py.py`, not in the server.

## 7. `server.yaml` delta (54 added, 3 removed) [diff]

| key | upstream | fork | why |
|---|---|---|---|
| `keepalive_frequency` | absent | `4.0` | §4 |
| `warnings.motion_capture.warning_if_rate_outside` | `[80.0, 120.0]` | `[40.0, 60.0]` | Motive streams at 50 Hz here, so the old band warned on every launch (`Motion capture rate off (Avg: 50.3)`) and trained everyone to ignore it. **Rig-specific — re-derive from your own Motive rate; a 100 Hz rig keeps upstream's band.** |
| `warnings.communication.publish_stats` | `false` | `true` | feeds `/cfX/connection_statistics`, which is how link stalls are diagnosed; costs one small message per drone per second. The counter-semantics comment block in the yaml is the **corrected** version (CLAUDE.md records that an earlier one was wrong). |
| `telemetry_watchdog_s` | absent | `5.0` | §1; `0` disables, and so does omitting the key |
| `firmware_params.query_all_values_on_connect` | `True` | `true` | cosmetic |
| comment on `cf231` | none | explains it is an upstream placeholder, not a drone | stops the next person hunting for a sixth drone |

`keepalive_frequency` and `telemetry_watchdog_s` are top-level under
`/crazyflie_server: ros__parameters:` — **not** under `warnings:`.

## 8. `launch.py` delta (26 added, 3 removed) [diff]

- `trace_cf` launch argument (default `''`) wired to `debug.trace_cf` (§5).
- `server` launch argument (default `True`). **`server:=False` starts mocap, RViz and the preflight
  GUI and no server** — no radio owner, nothing armable. That is the state
  `scripts/sync_initial_positions.py` requires, which before 2026-10-02 took a
  launch–sync–relaunch cycle.
- The three server `Node`s (`cflib`, `cpp`, `sim`) changed from
  `LaunchConfigurationEquals('backend', X)` to
  `IfCondition(PythonExpression(["'", backend, "' == X and '", server, "'.lower() in ('true','1')"]))`.
  **Trap when re-applying on a newer `launch.py`: keep all three conditions in step.** Missing one
  makes `server:=False` still start that backend — i.e. a radio owner you did not expect while you
  are rewriting `initial_position`.

## 9. Library changes the server depends on (not in `crazyflie_server.cpp`, but required)

All under `src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/`. **[diff]**

| API the server uses | defined in | what it is |
|---|---|---|
| `Crazyflie::setTrace(bool)` | `Crazyflie.h` → `Connection::setTrace` | trace switch (§5) |
| `Crazyflie::logReset(unsigned timeout_ms, size_t tries)` → `bool` | `Crazyflie.cpp` | bounded probe. The original no-arg `logReset()` stays **unbounded on purpose** (first handshake of connect) and now matches by `answersLogReset` |
| `LogBlock` / `LogBlockGeneric` ctor arg `timeout_ms` (0 = unbounded) | `Crazyflie.h` | recovery passes `recovery_timeout_ms_` |
| `LogBlock::start(period, timeout_ms)` — two-arg, **no default argument** — plus the one-arg forwarder | `Crazyflie.h` | **Trap: do not give it a default argument and do not collapse the forwarder.** Bounding CONNECT would let one busy link kill all five drones' launch, because a throw out of a log-block builder is not caught at the `CrazyflieROS` construction site. (The one-arg form forwards `m_timeout_ms = 0`; tier 1 originally used it and so ran the unbounded wait — the trap `627ab55` closed for uploads, left open one function away.) |
| `LogBlock::abandon()` | `Crazyflie.h` | skips the stop handshake in the destructor for blocks already gone from the drone |
| `crtpLogControlResponse::isAnswerTo(p, cmd, blockId)`, `isAnswerToCommand(p, cmd)`, enum `crtpLogControlCommand` | `crtp.h` | identity matching. **`logReset` is matched on the command ONLY** — its request carries no block id, so `data[1]` of the reply is indeterminate and matching an id there would reject the drone's own answer |
| memory-write reply matched by memory id + 32-bit address; non-zero status **throws**; 500 ms × 2, 3 resends, 10 s/chunk, 90 s/call | `Crazyflie::uploadTrajectory` | §2 |
| `registerLogBlock` / `unregisterLogBlock` guards | `Crazyflie.cpp` | `erase(end())` on a missing id was UB; plus constructor rollback if a `LogBlock` ctor throws |
| trace counters | `Crazyflie.cpp processPacket`, `CrazyradioThread.cpp`, `ConnectionImpl.h`, `Connection.cpp/.h` | §5 |

### Reply matching by identity, not shape — the single most important library change

`crtpLogControlResponse::valid()` and `crtpMemoryWriteResponse::valid()` test only port, channel
and payload size, which **every** reply of that kind satisfies. Matching on command byte + block id
(log), or memory id + address (memory), is exact, because the firmware answers by **reusing the
request packet**: `log.c logControlProcess` overwrites only `data[2]` and `size`, and
`crtp_mem.c memWriteProcess` carries the comment *"Dont' touch the first 5 bytes, they will be the
same."*

Two concrete failures this fixed **[CLAUDE.md]**:

1. Tier 1 is literally `stop(); start(p);`, so a late **stop** answer satisfied the **start** wait.
   Tier 1 then "worked", logged `log blocks restarted`, and the watchdog never escalated to the
   tier 2 that actually fixes it.
2. A resent upload chunk could leave the host one reply out of phase for the rest of the upload, so
   a chunk whose request was lost counted as written and `success=true` was reported for a
   trajectory with a 24-byte hole. `sizeof(poly4d)` is 132 and `132 % 24 != 0`, so the hole lands
   mid coefficient-array and the drone flies a hybrid of two polynomials.

**If reverted, both failures are silent.** Both are portable and are the strongest upstream
candidates. Tightening the memory predicate removed an accidental latency absorber, so a stale
reply now counts as proof the drone is alive-but-backlogged and buys more time, under the per-chunk
and per-call caps.

**Trap: do not "fix" `requestMemoryToc`.** An audit claimed it consumes GetInfo replies
positionally with no id check. That reading is inverted: `Crazyflie.cpp:19` is an unconditional
`#define FIRMWARE_BUGGY` with no `#undef`, so the positional branch is preprocessed away and the
live branch is `if (res::id(p) != i) { warn; --i; }` **[CLAUDE.md]**. Dropping the `--i` removes the
only absorber for a duplicated request, and indexing `m_memoryTocEntries[res::id(p)]` would be an
unbounded indexed write driven by a byte the drone chose — heap corruption in the process that owns
the only radio.

**Still unbounded, same trap:** most other `waitForResponse()` calls (params, log TOC). They run at
connect, where blocking is the documented behaviour — but **bound them before calling any of them
from a service handler.**

## 10. Re-apply checklist on a newer upstream

1. `git fetch upstream` and diff the three first-party files and the `deps/` directory (command at
   the top). Expect conflicts in `CrazyflieROS` log-block construction: the builder refactor
   touches every block.
2. Apply in this order, so each step compiles and is behaviour-neutral until the last:
   library API (§9) → builder refactor (`add_log_builder`, periods) → state and atomics (§3) →
   `on_logging_*` stamps → watchdog functions (§1) → upload + bad-trajectory set (§2) →
   keep-alive, trace, yaml, launch.
3. Verify: `./scripts/build.sh` with **no new warnings** under `src/crazyswarm2` (CLAUDE.md: any new
   warning there is a real regression). A sim run (`backend:=sim`) cannot exercise any of this —
   the sim server is a different binary. Then hardware, props off, on the floor: provoke a stall
   (kill a drone's log blocks by interrupting its link for >1 s) and confirm `/cfX/pose` returns
   ~1 s after the stall is declared.
4. Acceptance criterion for the OPEN fleet-wide-callback-group refactor, if it is ever done: while
   one drone rebuilds, the other four keep publishing `/cfX/pose` at 10.0 Hz and their `land` and
   `arm` still answer. Measure first — if the freeze stays ~1.0 s, take the cheap fix instead
   (lower `RECOVERY_TIMEOUT_MS` toward 300–500 ms) at zero concurrency risk.

## 11. What to offer upstream, in order

1. Identity-matched replies + bounded, resent, status-checked upload + `success`/`message` +
   the bad-trajectory refusal (§2, §9). Portable, safety-relevant, rig-independent.
2. The registry/builder refactor + the watchdog, behind `telemetry_watchdog_s: 0` (default off).
3. The trace hooks.
4. The `server:=False` launch argument.

**Keep fork-only:** the mocap-z gate wiring if upstream is not mocap-centric, the 40–60 Hz mocap
warning band, and `publish_stats: true`.

## Known gaps in this document

Stated honestly rather than papered over:

- Per-group added-line counts are approximate (~170 for the watchdog functions, ~60 for the builder
  refactor). Only the per-file totals (445/22, 54/3, 26/3, 512/68) are exact. Recompute a hunk with
  `git diff upstream/main HEAD -- <file> | grep -c '^+'` over the range you care about.
- CLAUDE.md attributes `add_on_set_parameters_callback`, LED-on-connect,
  `query_all_values_on_connect` and the foxglove/preflight launch nodes to this fork, but all four
  are already at `upstream/main` (`9ed4bae`). **This document did not diff against IMRCLab**, so it
  cannot say which are fork-original versus AI-DA-STC-original.
- The hardware claims (cf5 stalls, ~168 rx/s, ~1.0 s recovery, the two 2026-10-05 upload incidents)
  are cited from CLAUDE.md and the yaml comments. They were **not re-measured** while writing this.
- The `Crazyflie.h` / `Crazyflie.cpp` diffs were not read line by line — only added lines matching
  key terms. The §9 table may omit minor edits; the +266/−68 in `Crazyflie.h` includes
  constructor/ordering changes not enumerated here.
- The §3 gap was checked 2026-10-08 by grep: the stream handlers do not latch, the escort's initial
  `cf.takeoff` does. Not tested on hardware that the watchdog stays quiet during an escort run.
