# Workspace notes — fork-local

Notes for whoever maintains this fork (`Darkuantum/CrazySwarm2-with-Mocap`).
Rig and code knowledge lives in `CLAUDE.md`; this file covers the fork itself:
how it relates to AI-DA-STC, what is still owed upstream, and how to hand the
machine back.

## Branches

Only `main`, as of 2026-10-08. `local-fixes`, `workspace-notes` and `escort`
were merged in and deleted; their commits stay reachable from `main`'s history.
`escort` was deleted on 2026-10-08 at `627b499`, which was by then the merge
base with `main` -- it held nothing `main` did not, and `main` was 35 commits
ahead of it. Topic branches are expected to be short-lived here: `main` is the
trunk, and a branch that has stopped being ahead of it is finished.

`main` is **the fork's working trunk, not a mirror of AI-DA-STC.** It carries
fork-only work (the mission console, `crazyflie_shows`, a fork-specific
`CLAUDE.md`) on top of upstream. So **never open a PR from `main`**: cut a branch
from `upstream/main` and cherry-pick only the commit being proposed.
For a whole deliverable (the console, the show package) rather than one commit,
branch from `upstream/main` and copy the paths over with
`git checkout main -- <path>`; the strategy, the `pr/*` and long-lived `donate/*`
branch naming, the pre-donation checklist and the exact command sequences are in
[docs/CONTRIBUTING-UPSTREAM.md](docs/CONTRIBUTING-UPSTREAM.md). A `donate/*`
branch, if one is created, must be recorded in this section as deliberate and
deleted the day its PR merges or is abandoned. What the fork changed in the C++
server versus upstream is [docs/SERVER-CHANGES.md](docs/SERVER-CHANGES.md).

## Owed upstream: the sim trajectory arity fix

`backend:=sim` crashes on the first trajectory on a stock AI-DA-STC checkout:
two cffirmware generations disagree on `plan_start_trajectory`'s argument list,
and calling one with the other's raises `TypeError` inside a service callback,
killing the sim server. The fix resolves the binding's arity once at import and
dispatches. It is portable and nothing rig-specific.

`main` has it (inside `0095092`, next to unrelated fleet changes), so this fork
is fixed. The clean standalone form is **`e681554`**, which cherry-picks onto
`upstream/main` without conflict (checked 2026-09-19). To propose it:

```bash
git fetch upstream
git switch -c pr/sim-arity-fix upstream/main
git cherry-pick e681554
git push origin pr/sim-arity-fix        # then open the PR on GitHub (no `gh` here)
```

Not opened yet — the user deferred it on 2026-09-19. Do it before the laptop
goes back, or the next person hits the same crash with no trace of the fix.

## Remote layout

`origin` is the personal fork over SSH; `upstream` is AI-DA-STC over HTTPS and
is **fetch-only** — its push URL is deliberately `DISABLED-open-a-PR-instead`
(see `CLAUDE.md`). `main` tracks `origin`, so a bare `git push`/`git pull` never
touches AI-DA-STC. Reading new upstream work is explicit, and since `main` has
diverged it is a merge, not a fast-forward:

```bash
git fetch upstream && git merge upstream/main
```

Tags worth knowing: `archive/swarm-shows-714affe` is the original history of the
out-of-tree show repo that was folded into `crazyflie_shows/` (see that
package's `PROVENANCE.md`) — it is on `origin` as well as local (`git ls-remote --tags origin`
printed it 2026-10-08), so it is no longer the only copy.

## Restoring this clone to a stock checkout

If the working copy stays on a machine being handed back, undo the fork wiring
so the next user does not inherit an `origin` pointing at a personal fork over
an SSH key that no longer exists (every pull fails with `no such identity`, and
their commits get misattributed):

```bash
git remote set-url origin https://github.com/AI-DA-STC/CrazySwarm2-with-Mocap.git
git remote remove upstream
git config --local --unset user.name
git config --local --unset user.email
git config --local --unset core.sshCommand
git fetch origin                         # origin is now AI-DA-STC
git switch -C main origin/main           # local main = AI-DA-STC main; fork work stays on the fork
```

Do this *after* pushing all work and the archive tag to the fork, and *before*
deleting the keyfile (`~/near-intern/ssh/id_ed25519`) the remotes depend on.
