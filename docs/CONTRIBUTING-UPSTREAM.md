# Giving work back to AI-DA-STC without polluting it

**Audience:** the fork owner. **Scope:** how to hand `crazyflie_shows` (the unit that
contains `escort_show`) and `console/` to **AI-DA-STC/CrazySwarm2-with-Mocap** after
polishing, while development continues here.

Companion to `WORKSPACE-NOTES.md`, which holds the standing branch rules and the worked
example of the sim-arity fix still owed upstream (`e681554`).

**Evidence tags used below:** **[measured]** = command run in this repo on 2026-10-08,
the command is shown; **[source]** = read in code or in a doc in this tree;
**[knowledge, not re-verified]** = GitHub platform behaviour recalled, not looked up this
session — listed again under *Known gaps*.

---

## 0. Hard constraints — read before typing any git command

These are not style preferences. Each one has already cost something, here or nearby.

1. **`upstream` is FETCH-ONLY. Never push to it.** Its push URL is deliberately set to the
   bogus `DISABLED-open-a-PR-instead` so `git push upstream …` dies locally before it
   touches the network or any credential. **Do not "fix" that URL.** Verify it is still
   broken before any session of git work:

   ```bash
   git remote -v | grep upstream        # the (push) line must read DISABLED-open-a-PR-instead
   ```

2. **Another git user on this laptop holds credentials that CAN push to AI-DA-STC.** This is
   why the guard above exists: a stray push may *silently succeed* and land unreviewed code
   on the team's `main` instead of being rejected. The disabled push URL does not cover
   every route, so also never run:
   * `git push <any AI-DA-STC URL>` (bypasses the remote name entirely),
   * `git push --all` or `git push --mirror` (these iterate over **every** remote),
   * any push at all while a different `user.*` or credential helper is active.

3. **Never open a PR from `main`.** `main` is this rig's working trunk: it carries
   `CLAUDE.md`, `runbooks/`, rig-measured constants and 61 commits of fork-only work. A PR
   from it would propose all of that to the team. Every donation branches off
   **`upstream/main`**, never off `main`.

4. **`origin` (Darkuantum fork) is the only push target.** Pushes are SSH, keyed by
   `core.sshCommand`; the commit identity is already `darkuantum`.

5. **`gh` is not installed on this box** **[measured:** `command -v gh` → not found**]**.
   Every pull request in this document is opened **in a browser**. Nothing here needs the
   GitHub CLI.

---

## 1. The facts that decide the answer

All **[measured]** on 2026-10-08 against `upstream/main` = `9ed4bae`.

| Fact | Value | Consequence |
|---|---|---|
| Divergence | `git rev-list --left-right --count upstream/main...HEAD` → **0 behind, 61 ahead**; merge-base *is* `9ed4bae` | Upstream has not moved since the fork point. A branch cut from `upstream/main` is trivially mergeable **today** — that will not stay true forever |
| `console/` | `git ls-files console \| wc -l` → **14** tracked files; upstream has no `console/` | Pure addition, one directory, no upstream file touched |
| `console/backups/` | `git ls-files console \| grep -i backups` → **nothing tracked** | The rig's YAML backups are not in git. Re-run that grep before the PR anyway (checklist §5) |
| `crazyflie_shows/` | `git ls-files src/crazyswarm2/crazyflie_shows \| wc -l` → **32** tracked files | Pure addition under `src/crazyswarm2/`, where the other packages live |
| `escort_show` is not separable | it imports `swarm_show`, `constellation_show`, `preflight`, `abort`, `safety`, `constellation`, `escort`, `escort_teleop`, `escort_viz` **[source]** | **`escort_show` cannot be donated alone.** The deliverable is the whole `crazyflie_shows` package |
| Package deps | `package.xml` declares `rclpy`, `crazyflie_py`, `crazyflie_interfaces`, `geometry_msgs`, numpy, yaml (+ matplotlib exec) **[source]** | An ordinary `ament_cmake` package; nothing exotic to negotiate |
| **Undeclared dependency** | `escort_show.py:150` does `from motion_capture_tracking_interfaces.msg import NamedPoseArray`; `package.xml` declares no such depend **[measured:** grep + cat**]** | Builds fine, then fails at import on a clean checkout. **Fix before the PR** |
| Fork-only server/client changes | `crazyflie_server.cpp` 445 added / 22 removed, `server.yaml` 54/3, `launch.py` 26/3 (`git diff --numstat upstream/main HEAD`; the +467/+57/+29 figures quoted elsewhere are added-plus-removed sums), plus `crazyflies.yaml` and `config.rviz` edits. Full list: `docs/SERVER-CHANGES.md` | The shows do **not** need these to build — but see the safety note in §5 |
| Rig-specific constants | `crazyflie_shows/safety.py` carries `ARENA_CENTRE`, `ARENA_RADIUS_TESTED`, `ARENA_RADIUS_LOST` measured for **this room** (`ESCORT.md` §4, 8067 samples, 2026-10-01) **[source]** | An upstream user inherits our geofence. The single biggest open design question (§5) |
| Fleet addresses leak into donated docs | `grep -rln E7E7E7E7 console src/crazyswarm2/crazyflie_shows` → `SHOW_GUIDE.md`, `HANDOVER.md`, `console/mission_console/catalog.py`, `console/mission_console/static/app.js` **[measured]** | House rule: no roster in prose. Audit these four before donating |
| Upstream CI | repo-root `.github/` does **not** exist upstream; the workflows under `src/crazyswarm2/.github/` are vendored crazyswarm2's own **[measured:** `git ls-tree -r upstream/main --name-only`**]** | No repo-level CI will gate the PR. No `CONTRIBUTING` file was found either — ask the maintainers what they want |
| Show git history | already a fold-in of a retired repo (`PROVENANCE.md`, `archive/swarm-shows-714affe`) **[source]** | Preserving per-commit history upstream buys little; the messages are written for this rig. Donate as a squashed, re-written commit |
| `git filter-repo` | not installed **[measured]** | Any history surgery must use stock `git subtree split` |

---

## 2. "A mirror repo that edits its own upstream as a branch" — what GitHub actually does

Answering the idea directly. **[knowledge, not re-verified]**

* **A fork already *is* that thing.** A fork is a linked repository, and a **branch on your
  fork is exactly "a copy that edits its own version of upstream as a separate branch"**.
  The merge path is a pull request from `Darkuantum:<branch>` → `AI-DA-STC:main`. You do
  **not** need a second repository to get what you described.
* **No fork can push into its parent.** Only accounts with write access to AI-DA-STC can
  write to AI-DA-STC. A PR is a *request*; they merge it or they don't. This is the same
  boundary the `DISABLED-open-a-PR-instead` push URL enforces locally.
* **`git clone --mirror` pushed to a new repo is NOT a fork.** It has no fork-network link,
  so GitHub will not offer a cross-repo PR back to the original — you could only hand them
  a patch or a remote to pull from. It also carries all 61 fork-only commits.
  **Do not use `--mirror` for this**, and note it is on the forbidden list in §0 anyway.
* **One fork per account per upstream.** You already have it. A second "clean" fork would
  have to live under a different account or org.
* **A PR is a live lane.** Pushing more commits to the same branch updates the open PR. So
  the donation branch doubles as your "keep developing in the open" lane — accepting that
  unmerged work on it is visible to them.

**Verdict:** as literally described, a mirror that writes to upstream branches is not
possible and not desirable. The real equivalent is **option (b) below: a branch on this
fork, cut from `upstream/main`, opened as a PR.**

---

## 3. The options, compared honestly

| | (a) One self-contained PR per deliverable | (b) Long-lived donation branch, PR'd when ready | (c) Separate repo (submodule / subtree) | (d) "Mirror that edits upstream" |
|---|---|---|---|---|
| **Self-contained?** | Yes by construction — branch = upstream + that deliverable | Yes, same construction, but the branch persists | The repo is; **upstream would need a second repository to build** | n/a |
| **Pollution of upstream** | Minimal: one reviewable diff, no fork-only files | None until the PR opens; a draft PR shows work early | **Highest**: `.gitmodules` + pinned SHA in their `main`, or a subtree squash commit; and they must agree to depend on *your personal repo* | n/a |
| **Can you keep developing here?** | Yes on `main`, but branch and `main` diverge immediately and must be re-synced by hand | **Best**: a persistent "what upstream would receive" lane next to `main`; cost is porting edits between two copies | Easiest in isolation — but `crazyflie_shows` imports `crazyflie_py` and reads `crazyflies.yaml`, so you'd develop against a workspace checkout anyway, now split across two repos | n/a |
| **Merge path** | They merge; you `git fetch upstream && git merge upstream/main`. Expect **add/add conflicts** on donated paths | Same, plus "push to the same branch" for review rounds | **None.** You are asking them to link to you; ownership never transfers unless you move the repo into their org | **None** |
| **Fits this repo's conventions?** | Yes | Yes if scoped — see caveat | **No.** CLAUDE.md's vendoring rule ("a clone is a byte-for-byte copy of the rig", `src/` committed) argues directly against submodules | No |
| **Caveat** | For 28 interleaved show commits, cherry-picking is impractical — use `git checkout main -- <paths>` | `WORKSPACE-NOTES.md` wants topic branches short-lived (the `escort` branch was deleted 2026-10-08 as stale). A long-lived one is fine **only if** it is named `donate/*`, recorded in the Branches section as deliberate, and deleted the day the PR merges or is abandoned | Reasonable **for `console/` only** — it is not a colcon package and nothing in `src/` imports it | — |

**On cherry-pick vs path checkout.** The `WORKSPACE-NOTES.md` recipe (`git cherry-pick`)
is right for a single surgical commit like `e681554`. For the console (8 commits) and the
shows (28), the commits are interleaved with rig-specific edits, so **copy the files with
`git checkout main -- <path>` and make one fresh commit** on the donation branch.

---

## 4. Recommendation

| Deliverable | Recommended | Why it differs from the other |
|---|---|---|
| **`console/`** | **(a) one self-contained PR**, branch `pr/console`, cut from `upstream/main`, opened once the console is polished | 14 files, one directory, zero coupling to `src/`. The smallest and safest diff in the repo — easy to review, easy for them to decline with no side effects. Later fixes go in a second small PR. It needs no safety review |
| **`crazyflie_shows/`** (contains `escort_show`) | **(b) a long-lived `donate/shows` branch**, opened as a **draft PR** early, marked ready after polish | Not separable (§1), still under active development, and it needs real un-rig-ing work (arena constants, `package.xml`, roster leakage) that should not be done on `main`. A persistent lane lets that work happen in the open, under review, without touching the trunk |

**Order: console first, shows second.** The console is small, builds trust with the
maintainers, and needs no safety discussion. The two branches are independent, so neither
blocks the other.

**Before either: open the `e681554` sim-arity PR.** It is already owed (see
`WORKSPACE-NOTES.md`), it is a one-commit cherry-pick, and it proves the PR route works end
to end before anything larger is at stake.

Use `donate/*` naming for the shows lane. (An earlier draft of this plan used
`upstream/shows`; **do not** — a local branch named `upstream/…` is easy to confuse with
the `upstream` remote in every `git log` and `git push` you type afterwards.)

---

## 5. Polish still needed before donating

### Both deliverables

* [ ] **No rig data in the diff.** `git ls-files console | grep -i backups` (expect nothing —
      measured clean 2026-10-08) and check the one tracked file under
      `src/crazyswarm2/crazyflie_shows/data`.
* [ ] **No fleet roster in prose** (house rule — prose copies of the roster have drifted six
      times here and caused real incidents). Four files currently contain radio addresses:
      ```bash
      grep -rn "E7E7E7E7" console src/crazyswarm2/crazyflie_shows
      ```
      Replace each with a pointer to `./scripts/scan_fleet.sh --list`.
* [ ] **Repoint fork-only links.** READMEs and docstrings that link to `CLAUDE.md`,
      `runbooks/`, `WORKSPACE-NOTES.md` or `docs/` will dangle upstream. Remove or rewrite.
* [ ] **Build on the clean branch, not on `main`**: `./scripts/build.sh crazyflie_shows`.
* [ ] **Neutral commit identity** (`darkuantum`, already set locally) and **do not** ship
      this fork's `CLAUDE.md`.

### `crazyflie_shows`

* [ ] **Declare `motion_capture_tracking_interfaces` in `package.xml`.** `escort_show.py:150`
      imports it; the manifest does not list it (§1). Whether upstream's tree even provides
      that package was **not checked** — confirm before the PR.
* [ ] **Un-rig the arena.** `safety.py`'s `ARENA_CENTRE` / `ARENA_RADIUS_TESTED` /
      `ARENA_RADIUS_LOST` are *this* room, measured 2026-10-01. Either make them a launch /
      config parameter, or keep the constants and **fail loudly on the shipped default** with
      a banner telling the operator to re-measure. This is the one decision that needs the
      AI-DA maintainers' input, not ours — raise it in the draft PR rather than guessing.
* [ ] **Carry the containment watchdog across.** The `escort_show` containment check
      (`escort_show.py` ~line 884) watches every flying drone's **actual** pose, and exists
      because of the **2026-10-07 fly-away**: the VIP had a geofence and a staleness
      watchdog, the Crazyflies had neither and `self_stale_s` was dead config. Verify on the
      donation branch that (i) the watchdog is present, (ii) `self_stale_s` is actually read,
      and (iii) it covers the `adversary:=external` drone too. Donating the pre-fix state
      would be donating a known fly-away.
* [ ] **Re-check the flight status before writing it down.** As of 2026-10-08 `ESCORT.md`
      carries a dated "what flew" section (flown 2026-10-07, shown to guests 2026-10-08,
      walking case NOT cleared). Read it and `runbooks/ESCORT.runcard.md` at PR time and
      state exactly what they say. Getting this wrong in a PR is a safety claim.
* [ ] **Resolve or publish `ESCORT.md` "Decisions still open".** Most of all #1: with the
      shipped defaults `plan_escort --sweep` clears the ring only up to a **0.20 m/s** walk,
      against a normal walk of 1.0–1.4 m/s. Upstream must be told this, not left to find it.
* [ ] **Say what a stock upstream server loses.** The fork's `UploadTrajectory.srv` (+6:
      `success`/`message`) and `crazyflie_py/crazyflie.py` (+14: raises on a failed upload)
      are what make a bad upload *loud*. The shows guard with
      `getattr(result, 'success', True)`, so on a stock server **a failed upload is silent
      again**. That is a safety regression to state in the PR body — and optionally a third,
      separate, small PR.
* [ ] **Decide what docs travel.** The package's own design docs (`SHOW_GUIDE.md`,
      `CONSTELLATION.md`, `ESCORT.md`, `HANDOVER.md`, `PROVENANCE.md`) live inside the
      package precisely so it stays donatable — send them. The operator `runbooks/` are rig
      procedure; **default is to leave them on `main`.**

### `console/`

* [ ] **Document the restart-after-code-change caveat prominently.** The Python backend is
      loaded once while HTML/JS/CSS are re-read per page load, so a reloaded page can call
      routes an old backend lacks. This already produced a **real incident** — a teammate
      reported the web E-STOP failing with "not found"-type wording, and the e-stop did not
      fire. The guards exist (`/api/bootstrap` `code_changed` banner, 404-as-"outdated
      console", e-stop fallback to `/api/run`); `console/README.md:166` covers it. Make sure
      that text survives the trip and is not buried.
* [ ] **State the workspace assumption.** The console discovers flight scripts under
      `install/` and drives the `ros2` CLI, so it expects to sit inside this workspace. Say so
      at the top of its README.
* [ ] **Keep the decoupling rule in the donated README**: not a colcon package, nothing in
      `src/` imports it, `rm -rf console/` removes the feature and changes nothing else. That
      property is the whole argument for accepting it.
* [ ] **Keep the "`ros2` CLI, never in-process rclpy" rationale.** It reads like a quirk; it
      is a workaround for this rig's DDS discovery stall, and the maintainers need the reason
      or they will "modernise" it into a hang.

---

## 6. Command sequences

**Preconditions.** You are in `/home/jeremy/CrazySwarm2-with-Mocap`. The working tree may
have uncommitted changes (the runbook move, doc edits) — **commit or stash them on `main`
first**, or `git switch` carries them onto the donation branch. Re-read §0 before typing.

### 6a. The owed sim-arity fix (do this first)

```bash
git fetch upstream
git switch -c pr/sim-arity upstream/main
git cherry-pick e681554            # single, surgical commit
git push -u origin pr/sim-arity
```

### 6b. Console — option (a)

```bash
git fetch upstream
git switch -c pr/console upstream/main     # NOT from main
git checkout main -- console               # bring the files, not the history
git status --short                         # expect only console/** added
git ls-files console | grep -i backups     # expect NOTHING
grep -rn "E7E7E7E7" console                # expect nothing after the polish pass
# clean up fork-only links in console/README.md here, then:
git add console
git commit -m "console: optional web front-end for the ros2 CLI (self-contained, removable)"
git push -u origin pr/console
```

### 6c. Shows — option (b)

```bash
git fetch upstream
git switch -c donate/shows upstream/main   # long-lived lane; record it in WORKSPACE-NOTES.md
git checkout main -- src/crazyswarm2/crazyflie_shows
# do the §5 un-rig-ing work ON THIS BRANCH, then build it clean:
./scripts/build.sh crazyflie_shows
git add -A src/crazyswarm2/crazyflie_shows
git commit -m "crazyflie_shows: carousel, constellation and escort demos (planned and verified before arming)"
git push -u origin donate/shows
```

### 6d. Opening the PR (browser — `gh` is not installed)

1. Open `https://github.com/Darkuantum/CrazySwarm2-with-Mocap` — GitHub offers
   **"Compare & pull request"** for a freshly pushed branch.
2. Set, explicitly, on the compare page:
   * **base repository:** `AI-DA-STC/CrazySwarm2-with-Mocap`, **base:** `main`
   * **head repository:** `Darkuantum/CrazySwarm2-with-Mocap`, **compare:** your branch
3. For `donate/shows`, choose **Create draft pull request**. Mark it ready after the §5
   checklist is clear.
4. Sanity-check the diff **on the compare page before submitting**: console ≈ 14 files,
   shows ≈ 32 files, **no deletions, no `CLAUDE.md`, no `runbooks/`**. If the file count is
   in the hundreds, you branched off `main` — stop and start over from `upstream/main`.

### 6e. Keeping both lanes in sync while development continues on `main`

```bash
# port one later improvement from main into the donation lane, by path:
git switch donate/shows
git checkout main -- src/crazyswarm2/crazyflie_shows/crazyflie_shows/escort.py
git commit -am "escort: <what changed>" && git push

# take an upstream change into the lane:
git fetch upstream && git merge upstream/main
```

**Do not rebase a branch with an open PR** — it rewrites commits reviewers have already
read. Merge instead.

### 6f. After AI-DA-STC merges

```bash
git fetch upstream
git switch main
git merge upstream/main          # expect add/add conflicts on the donated paths
# resolve per path: normally keep the merged upstream version, then re-apply main-only edits
git push origin main
git branch -d pr/console donate/shows
git push origin --delete pr/console donate/shows
```

Order matters: **merge the PR, then merge `upstream/main` into `main`, and only then resume
editing those paths on `main`** — otherwise the conflicts compound. Then update
`WORKSPACE-NOTES.md` "Branches" and drop the donated rows from "Owed upstream".

---

## 7. If you want a second repo anyway (option c — console only)

Only if AI-DA-STC says they would rather link than vendor. `git filter-repo` is **not
installed** here, so use stock git:

```bash
git subtree split -P console -b console-only     # history of console/ alone
git remote add console-repo git@github.com:Darkuantum/<new-repo>.git
git push console-repo console-only:main
```

The standalone repo's README must state that the console expects to sit inside a
crazyswarm2 workspace (it discovers flight scripts under `install/`).

---

## 8. What to put in the PR body

State, with evidence and in this order:

1. **What it is and what it touches.** "Pure addition: N files under `<path>`; no existing
   file in your tree is modified." Say it, because it is checkable and it is the reason to
   accept.
2. **What it depends on in this fork** (§1), and in particular that the shows' failed-upload
   refusal is **silent** on a stock server.
3. **What is rig-specific** — the arena constants above all — and the proposed way to
   parameterise them. Ask; do not decide for them.
4. **How to run it without hardware**: `ros2 launch crazyflie launch.py backend:=sim`, and
   `--ros-args -p use_sim_time:=true` for the trajectory demos.
5. **What has actually been flown on hardware versus sim only** — read `ESCORT.md` and
   `runbooks/ESCORT.runcard.md` at the moment of writing. Do not repeat a status from
   memory; the prose copies in this repo have drifted before.

---

## 9. Known gaps in this document

Stated plainly rather than papered over. Each is a thing to confirm, not a thing to assume.

* **GitHub platform behaviour** in §2 (fork/PR semantics, one fork per account, `--mirror`
  not creating a fork) is **[knowledge, not re-verified]** — not looked up this session.
  Confirm before relying on it.
* **AI-DA-STC's contribution conventions were not checked.** No `CONTRIBUTING` file and no
  repo-root `.github/` exist on `upstream/main` **[measured]**, but their preferences may
  live outside the repo. Ask before opening the shows PR.
* **Whether upstream's tree provides `motion_capture_tracking_interfaces`** was not checked,
  and `crazyflie_shows/package.xml` does not declare it.
* **The console's tracked files were checked for `backups/` and were clean** **[measured]**,
  but the full 14-file list was not read line by line for rig data. Do the audit in §5.
* **`escort_show`'s hardware-flight status** was inconsistent between sources until
  2026-10-08, when `ESCORT.md` gained a dated section. This document still asserts nothing about
  it: copy the status from `ESCORT.md` at PR time.
* **The post-merge conflict behaviour in §6f was reasoned, not tested.** A dry run in a
  scratch clone before the real merge is cheap and advisable.
* **Two things are not decided and need AI-DA maintainers' input:** how to parameterise the
  `safety.py` arena constants, and whether `runbooks/` should travel at all (this document
  recommends leaving them on `main`).
