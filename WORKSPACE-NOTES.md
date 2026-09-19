# Workspace notes — fork-local

Notes for whoever maintains this fork. Not upstream material: this branch
exists so these notes never appear in a PR diff back to AI-DA-STC.

## Branches

| Branch | What |
|---|---|
| `main` | tracks AI-DA-STC content exactly |
| `local-fixes` | the two local commits below |
| `workspace-notes` | this file only |

`local-fixes`:

- **`plan_start_trajectory` arity detection** — two cffirmware generations are
  in the wild. The older `planner.h` takes one `relative` flag; the newer
  splits it into `relative_position`/`relative_yaw` and wants the current yaw.
  Calling the old one with the new argument list raises `TypeError` inside a
  service callback, killing the sim server on the first trajectory. The SWIG
  wrapper is a plain Python function, so the arity is introspectable — resolve
  it once at import and dispatch. **Portable and upstreamable; nothing
  rig-specific.**
- **Rig calibration** — measured start positions for cf1/2/4/5/8 in the
  OptiTrack volume, plus an incidental 644→755 mode change. Rig-specific,
  drifts between sessions, deliberately kept out of the fix commit so that one
  stays cherry-pickable.

**`backend:=sim` crashes on the first trajectory while `main` is checked out.**
The fix lives only on `local-fixes`.

## Remote layout

`origin` is the personal fork over SSH; `upstream` is AI-DA-STC over HTTPS.
Every local branch tracks `origin`, so a bare `git push`/`git pull` never
touches AI-DA-STC. Reading new upstream work is explicit too — a fork does not
auto-sync:

```bash
git fetch upstream && git merge --ff-only upstream/main
```

## Restoring this clone to a stock checkout

If the working copy lives on a machine being handed back, undo the fork wiring
so the next user does not inherit an `origin` pointing at a personal fork over
an SSH key that no longer exists (every pull fails with `no such identity`,
and their commits get misattributed):

```bash
git remote set-url origin https://github.com/AI-DA-STC/CrazySwarm2-with-Mocap.git
git remote remove upstream
git config --local --unset user.name
git config --local --unset user.email
git config --local --unset core.sshCommand
git branch -D local-fixes          # safely on the fork
```

Do this *before* deleting any keyfile the remotes depend on, and push all work
first. The arity fix is worth a PR to AI-DA-STC before the machine goes back —
otherwise the next person hits the same crash with no trace of the solution.
