# PROVENANCE — where this package came from

`crazyflie_shows` was developed out-of-tree in `~/near-intern/swarm-shows/`, a
standalone git repo (one commit, `714affe`, never pushed anywhere), then copied
into this workspace to fly. On **2026-09-18** that repo was **subsumed** into this
fork and deleted. This package is now the only copy — edit it here.

Why: the two copies had already drifted once. The out-of-tree config still named
the retired cf10/cf12, and anything reading it (the old `mocap_verify.sh`,
`show_launch.py`, `plan_show`'s default) either hung the server or validated a
geometry that was not the one being flown.

## Recovering the original

The original history is kept in this repo as a local tag:

```bash
git log archive/swarm-shows-714affe                     # the original commit
git show archive/swarm-shows-714affe:README.md          # any file, as it was
git ls-tree -r --name-only archive/swarm-shows-714affe  # all 32 files
```

The tag is **local**. Push it with `git push origin archive/swarm-shows-714affe`
if it should survive a fresh clone (never to `upstream` — see `CLAUDE.md`).

## What moved where

| from `~/near-intern/swarm-shows/` | now |
|---|---|
| `crazyflie_shows/` | `src/crazyswarm2/crazyflie_shows/` (this package) |
| `deck_check.py` | `scripts/deck_check.py` |
| `complex-shows-report.html` | `reference/complex-shows-report.html` — the published "Beyond the Carousel" research report |
| — (claude.ai artifact only) | `reference/swarm-replay-sim-20260907.html` — the "Crazyflie Swarm Replay" of the 09-07 sim flight, full position trace embedded |
| `.claude/workflows/deep-research-swarm-shows.js` | `.claude/workflows/` (repo root) |
| `README.md` | not carried — it only described copying this package in; recoverable from the tag |

Both research artifacts are also still published under the user's claude.ai
account ("Beyond the Carousel", "Crazyflie Swarm Replay").

## What changed on the way in

- `config/crazyflies.yaml`: `robots:` block synced from the workspace yaml
  (cf1/cf2/cf3/cf5/cf8). Keep it in sync — `show_launch.py` hardcodes this file
  and `plan_show` defaults to it.
- `scripts/deck_check.py`: hardcoded the retired roster; now reads the enabled
  fleet from `crazyflies.yaml` (`--yaml` to override).
- `scripts/mocap_diag.sh`: config found relative to the script instead of a
  hardcoded `~/near-intern` path; section 8 runs the workspace's **vendored**
  mocap node instead of the apt one, which this rig no longer has.
- `scripts/mocap_verify.sh` and five dated diagnostic outputs: **removed**. It
  ran the apt node and aborted at its first check. Its questions are answered by
  `ros2 topic hz /poses`, `python3 scripts/sync_initial_positions.py --dry-run`
  (body names vs fleet), and the mission console's health graph.
- `HANDOVER.md` / `SHOW_GUIDE.md`: "deploy by `cp -r`" steps replaced with
  "build in place"; references to `mocap_verify.sh` repointed. The apt-era mocap
  sections of HANDOVER (the `141.23.110.162` address step) are otherwise left as
  history — the vendored driver made them obsolete.

## Claude context

- **Sessions:** all 11 transcripts from the old project folder
  (`~/.claude/projects/-home-jeremy-near-intern-swarm-shows/`), with their
  sidecar folders (tool results, subagent transcripts, the deep-research workflow
  run), were copied into this project's folder. They mention the old paths, since
  they are history. The originals were left in place.
- **Memory:** merged only what was still true (the show's design rules and
  status, the personal-git-identity setup, the separate AirStack project). Four
  memories were deliberately not carried because they had become false: the sim
  fix listed as uncommitted (committed in `0095092`), the `141.23.110.162`
  address workaround, a ~27 Hz `/poses` problem (50 Hz now), and a show status
  naming cf10/cf12 as never flown.

## Status at the time of the merge

Flown on hardware 2026-09-17: all five drones, seven figures, landed at
t+63.8 s. Separation sits at 99% of budget — 0.91 m against a 0.90 m floor
during counterflow — so run `plan_show` after every position sync. The research
report's first finding (downwash is an ellipsoid, not a sphere) is the obvious
lever for the next iteration: it would relax exactly that budget.
