"""2D STL pursuit-evasion 'game' flown on real Crazyflie hardware.

Ported 1:1 from `untitled15_2D_STL_2_5_4.m` (a MATLAB simulation of a Signal
Temporal Logic / zero-sum game between a defender team and an intruder
team), following the same format/conventions established by
`stl_pursuit_evasion_game_v2.py` (itself following `hello_world.py` and
`drone_rhombus_formation.py`): the high-level `cf.goTo(goal, yaw, duration)`
waypoint command streams a fresh reference position to every drone on every
control tick. This supersedes `stl_pursuit_evasion_game_v2.py` (which ported
the earlier `_2_5` revision) and adds the "_2_5_4" revision's changes
described below.

Mission overview (unchanged in spirit from the .m file):
  * 5 DEFENDER drones (D1-D5) are assigned roles (Goalkeeper / Interceptor /
    Chaser) from a fixed "detection accuracy" ranking, then use an STL
    ("Signal Temporal Logic") robustness gradient controller -- built on a
    softmax-approximated block-robustness cost matrix and a Hungarian
    (optimal) assignment between defenders and active intruders -- to
    intercept intruders before they reach their goals. The assignment cost
    is reweighted by each active intruder's URGENCY (how close it already is
    to its goal, and how fast it is currently moving), so a
    breaching/sprinting intruder is preferentially given its best available
    defender instead of being starved while defenders redundantly cluster on
    easier, already-covered intruders.
  * 5 INTRUDER drones (I1-I5) each independently: (1) follow a sequence of
    pre-planned waypoints that arc around the defender cluster and are
    re-planned on a receding horizon (also forced immediately, subject to a
    short cooldown, whenever a defender is detected sitting roughly between
    the intruder and its goal within repulsion range), (2) occasionally
    feint/deceive by briefly moving in a false direction, (3) approach from a
    fixed HIGH/CENTRE/LOW flank, (4) modulate their speed (crawl/base/sprint)
    based on how close the nearest defender is, and (5) evade defenders and
    teammates via potential-field repulsion.
  * An intruder is marked REACHED once within `GOAL_RADIUS` of its goal, or
    BLOCKED if a defender gets within `BLOCK_RADIUS` of it (or if it stalls
    for many consecutive steps). A BLOCKED intruder is commanded to land
    immediately, freeing up its airspace. The mission ends once every
    intruder has been resolved (REACHED or BLOCKED) or `MAX_STEPS` is hit, at
    which point any drone still flying lands too.

WHAT CHANGED IN THE "_2_5_4" PORT (relative to `stl_pursuit_evasion_game_v2.py`)
---------------------------------------------------------------------------
* TRUE STL-GRADIENT DEFENDER CONTROL (replaces the old ad hoc potential
  field). `stl_pursuit_evasion_game_v2.py` summed FOUR independently
  hand-tuned force fields for each defender (grad_block / grad_sep /
  grad_collision / grad_arena, each with its own gain) and only computed STL
  robustness afterwards, purely for logging -- robustness never actually
  drove the motion. In this port, each defender's conjunctive specification
      phi_di = phi_reach AND phi_sep AND phi_collision AND phi_arena AND phi_avoidIntr
  is scored as ONE smooth-min robustness via `soft_min` over every atomic
  predicate's robustness value, and the defender moves along the ANALYTIC
  GRADIENT of that robustness, obtained via the softmin "attention" weights
  `attn_w_i = exp(-SOFT_K*rho_i) / sum_j exp(-SOFT_K*rho_j)`. Because
  attn_w automatically concentrates on whichever predicate is closest to
  failing, hard collision avoidance emerges naturally (no separate
  collision-strength gain needed) whenever phi_collision becomes the
  worst-off term, and relaxes once clear. See `_defender_stl_gradient()`
  below.
* REACH-URGENCY CAP (fixes defender-defender / defender-inactive-intruder
  collisions). `rho_reach` (= -||target - pos||) is unbounded while every
  safety predicate is naturally bounded to about one radius, so feeding raw
  `rho_reach` into the same softmin as the bounded safety terms meant "reach"
  almost always dominated the attention weights -- safety terms never got
  enough weight to steer the defender until agents were already overlapping.
  `REACH_URGENCY_CAP` caps only the VALUE used for the attention weighting
  (the true unit-vector gradient direction toward the target is unaffected),
  so any safety predicate that drops below that cap now correctly wins the
  attention weighting and takes over the motion direction.
* NEW HARD PREDICATE `phi_avoidIntr`: keeps every defender clear
  (`INTR_KEEP_OUT`) of INACTIVE (already-blocked/reached, and therefore
  frozen) intruders that are not its own current target -- previously
  nothing stopped a defender's route from running straight through one.
* CHASER ROLE NOW ACTUALLY ASSIGNED. With 5 defenders exactly matching the 5
  `DETECTION_ACCURACY` entries, the .m file's role split
  (`goalkeeper = accRank(end)`, `interceptors = accRank([1,2,4])`,
  `chasers = accRank(3)`) gives one defender the Chaser role, so the
  `role == 3` branch (previously dead code, "kept for fidelity" only, in the
  3-defender v2 port) is now live.
* FULL TEAM SIZE: 5 defenders + 5 intruders (10 Crazyflies total: cf1-cf5 =
  defenders, cf6-cf10 = intruders), matching the .m file exactly, rather than
  the reduced 3v3 team `stl_pursuit_evasion_game_v2.py` used.
* LARGER SIMULATED ARENA, SAME REAL FLIGHT FOOTPRINT: the .m arena grew from
  30x30 to 100x100 (and radii/speeds grew to match, e.g. `blockRadius`
  2.2->5, `defSafeRadius`/`collisionRadius`->5), but this port keeps the same
  4m x 4m real flight arena as `stl_pursuit_evasion_game_v2.py` by using a
  smaller `SCALE` factor (`4/100` instead of `4/30`). Every spatial quantity
  is rescaled the same way `stl_pursuit_evasion_game_v2.py` did.
* !! UNVERIFIED FOR FLIGHT AT SPEED_BOOST > ~6: measured in simulation, the
  closest commanded approach between two drones at SPEED_BOOST = 31.25 is
  ~0.08 m, which is below a Crazyflie's ~0.09 m rotor-tip diameter. This is
  NOT an evasion-tuning bug -- it is inherent to boosting an explicit-Euler
  discrete-step simulation: a 0.02 m tick penetrates the fixed 0.20 m
  COLLISION_RADIUS / DEF_SAFE_RADIUS before the repulsion terms can ramp up.
  Measured min separation vs. boost is chaotic, NOT monotonic (6x -> 0.19 m,
  8x -> 0.15 m, 10x -> 0.006 m, 12x -> 0.12 m, 31.25x -> 0.08 m), so there is
  no safe boost value reachable by tuning this one knob. Note also that
  `rng = np.random.default_rng()` below is UNSEEDED, so each run differs and
  a single clean simulation run proves nothing.
  Root cause: a defender homes onto `target_pt = pos_ii` (its target's exact
  position) and phi_avoidIntr deliberately SKIPS active intruders, so nothing
  in the specification bounds defender-to-target separation from below. A
  naive standoff ring around the target does not fix this -- it prevents
  defenders reaching BLOCK_RADIUS (game ends 0 BLOCKED / 5 REACHED) and the
  violation simply migrates to other pairs. Proper fixes require re-tuning
  the safety geometry for the coarser timestep.
  VERIFY IN SIMULATION (see `animate_stl_v3.py`) BEFORE FLYING THIS.
* SPEED_BOOST (see the constant block below) uniformly scales every
  "metres per step" quantity so the mission flies at a visible speed. The
  straight geometric port moved 0.00064 m/step = 1.3 cm/s, needing ~6000
  steps (~300 s) just to cross the 3.84 m arena -- which is why earlier
  revisions of this file almost always ended via the `MAX_STEPS` cutoff with
  intruders still ACTIVE. At the default SPEED_BOOST = 31.25 (0.40 m/s) an
  intruder crosses the arena in ~192 steps / ~10 s, so the mission now
  resolves every intruder to REACHED/BLOCKED well inside the step budget.
* MAX_STEPS = 12000 is a backstop, not the expected runtime: 12000 * 0.05 s
  = 600 s, which exceeds a Crazyflie's usable battery. It should never be
  reached at the boosted speeds (games finish in a few hundred steps), but
  lower it if you want a hard wall-clock guarantee on the flight.

IMPORTANT HARDWARE / SIMULATION NOTES (carried over from v2)
--------------------------------------------------------------
* This script flies BOTH teams as real drones (10 Crazyflies total:
  `crazyflies[0:5]` = defenders, `crazyflies[5:10]` = intruders) and uses ONE
  SHARED CRUISE ALTITUDE (`CRUISE_ALTITUDE`) for every drone -- the original
  .m file is purely 2D (X/Y only).
* All plotting, STL-robustness logging for plots, per-agent robustness
  logging, and the animated-video export (Figures 1-6 / VideoWriter in the
  original .m file) have been DROPPED ENTIRELY, matching v2's convention.
  Only the flight-control logic and a final text mission summary are kept.
* The original .m arena is 100 m x 100 m, cornered at (0,0). This version
  targets the same real flight arena as v2: a 4 m x 4 m square
  (SCALE = 4/100 = 0.04) spanning X in [-2, +2] m and Y in [-2, +2] m, with
  the origin at the arena's centre (matching the mocap frame's origin)
  rather than at a corner. Every spatial quantity in the .m file (positions,
  goals, radii, speeds/step, offsets, distance thresholds, and
  distance-based numerical-safety floors) has been multiplied by SCALE, and
  every absolute *position* (initial positions / goals) has additionally
  been re-centred (shifted by half the original 100 m span before scaling)
  so the exact same *relative* geometry and discrete-step dynamics are
  preserved, just re-based to a centre-origin frame. A few quantities are
  intentionally left UNSCALED because they are dimensionless or are
  step-counts rather than distances -- see the inline comments next to each
  constant below for the reasoning behind every single value.
* Like v2, all *_SPEED constants carry an extra "* 0.8" hardware-safety
  derate on top of the geometric SCALE factor (a v2 convention, kept here
  for consistency). NOTE that the default SPEED_BOOST = 31.25 is exactly
  1/(SCALE * 0.8), so it cancels that derate out; set SPEED_BOOST = 25.0 to
  keep a real 20% margin (0.32 m/s instead of 0.40 m/s).
* The MATLAB script is a discrete-step simulation (no explicit physical time
  unit -- "speed" is really "metres moved per step"). Exactly like v2, each
  simulation step here is mapped to one `cf.goTo(..., DT)` hardware control
  tick with `DT = 0.05 s` (20 Hz), so the ported dynamics play out in real
  time at the same relative pace as the original simulation.
* `MAX_STEPS` steps * `DT = 0.05 s` bounds the game's wall-clock duration
  (see the MAX_STEPS note above). Reduce/raise `MAX_STEPS` for a
  shorter/longer test flight -- the control logic is unaffected either way.
  Because `cf.goTo(goal, yaw, DT)` plans a polynomial that decelerates to
  rest within each tick, the speed actually achieved in flight sits somewhat
  below the commanded `*_SPEED / DT` figure.
* Unlike the 3-defender v2 port, `DETECTION_ACCURACY` here has exactly 5
  entries for exactly 5 defenders, so no MATLAB index-growth quirk needs
  reproducing in `assign_defender_roles()` -- the role split is a direct,
  1:1 translation of the .m file's `accRank(...)` indexing.
"""

import math

import numpy as np
from crazyflie_py import Crazyswarm
from scipy.optimize import linear_sum_assignment

# ===================== Scale factor =====================
# Original .m arena: 100 m x 100 m.  Real flight arena: 4 m x 4 m (same
# footprint as stl_pursuit_evasion_game_v2.py), centred on the origin (X in
# [-2, +2] m, Y in [-2, +2] m).
SCALE = 4.0 / 100.0   # = 0.04

# ===================== Arena / team sizes =====================
ARENA_W = 100.0 * SCALE          # = 4.0 m
ARENA_H = 100.0 * SCALE          # = 4.0 m
NUM_DEFENDERS = 5
NUM_INTRUDERS = 5

DEFENDER_CF_NAMES = ['cf1', 'cf2', 'cf4', 'cf5', 'cf8']       # <- pick your actual names
INTRUDER_CF_NAMES = ['cf10', 'cf11', 'cf12', 'cf13', 'cf14']

# The arena is centred at the origin rather than cornered at (0, 0): all
# boundary/clamp logic below works against these min/max bounds instead of
# the [0, ARENA_W] x [0, ARENA_H] range the original .m file assumes.
ARENA_X_MIN = -ARENA_W / 2.0    # = -2.0 m
ARENA_X_MAX = ARENA_W / 2.0     # = +2.0 m
ARENA_Y_MIN = -ARENA_H / 2.0    # = -2.0 m
ARENA_Y_MAX = ARENA_H / 2.0     # = +2.0 m

# ===================== Initial positions (scaled + re-centred) =====================
# new = (original - 50) * SCALE  -- the "-50" re-bases the original 100 m
# corner-origin frame to be centred on the origin before scaling, so the
# same relative layout now sits inside X,Y in [-2, +2] m.
# Original (defPosInit, evaluated at arenaW=arenaH=100):
#   [[95,5],[92,20],[88,40],[85,60],[90,80]]
DEF_POS_INIT = np.array([
    [1.20, -1.75],
    [1.24, -1.01],
    [1.19, -0.16],
    [1.30, 0.64],
    [1.32, 1.32],
])

# Original (intruderPosInit): [[2,2],[2,25],[2,45],[2,70],[2,95]]
INTRUDER_POS_INIT = np.array([
    [-1.33, -1.55],
    [-1.35, -0.84],
    [-1.28, -0.18],
    [-1.25, 0.45],
    [-1.20, 1.08],
])

# Original (intruderGoals): [[98,10],[98,25],[98,45],[98,69],[98,90]]
INTRUDER_GOALS = np.array([
    [1.92, -1.60],
    [1.92, -1.00],
    [1.92, -0.20],
    [1.92, 0.76],
    [1.92, 1.60],
])

# Initial straight-line distance from each intruder's start to its goal.
# Used to measure how much progress (0->1) an intruder has made, which
# feeds into the urgency-weighted defender assignment.
INIT_GOAL_DIST = np.linalg.norm(INTRUDER_POS_INIT - INTRUDER_GOALS, axis=1)

# ===================== Speed boost (per-step velocity scale) =====================
# A straight geometric port (speed = 0.02 * SCALE * 0.8 = 0.00064 m/step at
# DT = 0.05 s) flies at 1.3 cm/s -- it takes ~300 s / ~6000 steps just to
# cross the 3.84 m arena, which looks frozen on real hardware. SPEED_BOOST
# uniformly speeds up the discrete-step dynamics without changing any of the
# tuned *geometry* (radii, zones, thresholds all stay at their SCALE values).
#
#   effective speed = 0.02 * SCALE * 0.8 * SPEED_BOOST / DT   [m/s]
#                   = 0.00064 * SPEED_BOOST / 0.05
#
#   SPEED_BOOST = 31.25  ->  0.400 m/s  (0.8 derate fully cancelled)
#   SPEED_BOOST = 25.00  ->  0.320 m/s  (keeps the 0.8 hardware derate)
#   SPEED_BOOST =  1.00  ->  0.013 m/s  (original geometric port)
#
# CRITICAL: this factor MUST be applied to every "metres per step" quantity,
# not just the *_SPEED constants. The intruder's motion is
# `active_speed * grad_reach + 0.35 * grad_evade`, where grad_reach is a UNIT
# vector (so its magnitude is exactly active_speed) but grad_evade carries an
# absolute magnitude built from REPULSION_STRENGTH / CHANNEL_STRENGTH. Boost
# the speeds alone and the reach term out-scales the evade term, so intruders
# stop dodging defenders and drive straight through them. Every constant
# tagged "-> m/step" below is therefore boosted in lockstep.
SPEED_BOOST = 31.25

# ===================== Speeds / radii (all distances -> scale by SCALE) =====================
# Original .m (_2_5_4): intruderSpeed=defenderSpeed=0.02 (both raised
# relative to the arena's own scale vs. the previous revision), blockRadius
# raised 2.2->5. As in v2, an extra "* 0.8" hardware-safety derate is
# applied to every *_SPEED constant on top of the geometric SCALE factor.
INTRUDER_SPEED = 0.02 * SCALE * 0.8 * SPEED_BOOST     # -> m/step; = 0.0200 (0.400 m/s)
DEFENDER_SPEED = 0.02 * SCALE * 0.8 * SPEED_BOOST     # -> m/step; = 0.0200 (0.400 m/s)
BLOCK_RADIUS = 5.0 * SCALE                  # = 0.20
GOAL_RADIUS = 0.8 * SCALE                   # = 0.032
INTRUDER_SPEED_MAX = 0.020 * SCALE * 0.8 * SPEED_BOOST  # -> m/step; = 0.0200  sprint speed when path is clear
INTRUDER_SPEED_MIN = 0.005 * SCALE * 0.8 * SPEED_BOOST  # -> m/step; = 0.0050  crawl speed when threatened

# Dimensionless per-defender "detection accuracy" used only for ranking ->
# NOT a distance, so NOT scaled. Exactly 5 entries for exactly 5 defenders
# (unlike v2's 3-defender port, no MATLAB index-growth quirk applies here).
DETECTION_ACCURACY = np.array([0.65, 0.90, 0.95, 0.80, 0.70])

# ---- Urgency-weighted assignment tuning ----
# Dimensionless blend/gain coefficients -> NOT scaled.
URGENCY_GAIN = 1.6      # overall strength of the urgency bias
URGENCY_GOAL_W = 0.6    # weight on "how close to goal" within urgency
URGENCY_SPEED_W = 0.4   # weight on "how fast it's currently moving" within urgency

# ---- STL horizon & weights ----
STL_HORIZON = 20                       # step count -> NOT scaled
# soft_k multiplies robustness VALUES (which are distances) inside exp(k*x).
# Distances shrink by SCALE, so k must grow by 1/SCALE to keep the softmax
# approximation just as "sharp" (in relative terms) as the original.
SOFT_K = 5.0 / SCALE                   # = 125.0

# Dimensionless blend weights between rho_reach / rho_sep / rho_arena
# predicates -> NOT scaled (they weight already-normalised unit vectors,
# only their relative ratios matter).
ROLE_WEIGHTS = np.array([
    [0.3, 0.5, 0.2],
    [1.0, 0.3, 0.1],
    [0.7, 0.2, 0.1],
])

# ---- Defender tuning (distances -> scaled; dimensionless -> not) ----
DEF_SAFE_RADIUS = 5.0 * SCALE               # = 0.20  (raised from 1.8 in the previous revision)
INTR_SAFE_RADIUS = 5.0 * SCALE              # = 0.20
PINCER_SPREAD = 3.0 * SCALE
INTERCEPT_BIAS = 0.45                       # dimensionless blend (0-1) -> not scaled
LOOKAHEAD = 12                              # step-count cap on distance/speed ratio -> not scaled
REPULSION_RANGE = 5.0 * SCALE
REPULSION_STRENGTH = 0.25 * SCALE * SPEED_BOOST   # -> m/step; boosted with the speeds so
CHANNEL_STRENGTH = 0.20 * SCALE * SPEED_BOOST     #    evasion keeps its original authority
INTR_REP_STRENGTH = 0.12 * SCALE * SPEED_BOOST    #    relative to grad_reach
# LEGACY: gain of the old ad hoc defender-separation force. No longer used --
# the defender loop now derives its separation push from the analytic
# STL-gradient controller (role weight w[1] still applies there). Kept only
# for fidelity with the .m file's (unused) constant.
DEF_REP_STRENGTH = 0.08 * SCALE

# ---- Defender-defender hard collision avoidance ----
# phi_collision's robustness (||pos_di-pos_dj|| - COLLISION_RADIUS) is one of
# the predicates in each defender's smooth-min STL specification, so it is
# naturally enforced at FULL strength (unweighted by role) via the softmin
# gradient controller below -- no separate gain is needed any more.
COLLISION_RADIUS = 5.0 * SCALE              # = 0.20 (raised from 1.2)
# LEGACY: gain of the old ad hoc collision-avoidance push. Unused (kept only
# for fidelity with the .m file's (unused) constant).
COLLISION_STRENGTH = 0.50 * SCALE

# ---- Defender avoidance of already-neutralized (inactive) intruders (NEW) ----
# A defender chasing its own assigned target previously had no reason at all
# to avoid a DIFFERENT intruder that has already been blocked or has reached
# its goal (and is therefore frozen in place) -- nothing in its STL
# specification treated non-target intruders as obstacles. phi_avoidIntr
# fixes this: unweighted (hard), like phi_collision, so it applies
# regardless of role.
INTR_KEEP_OUT = 1.0 * SCALE                 # = 0.04  min clearance a defender keeps from an inactive intruder

# ---- STL attention-weight scale fix (defender-defender collisions) ----
# rho_reach = -||target-pos|| is UNBOUNDED (can be many metres), while the
# safety predicates (rho_sep, rho_collision, rho_arena, rho_avoidIntr) are
# naturally bounded to within about one radius of violation. REACH_URGENCY_CAP
# caps only the VALUE used for the attention weighting (the unit-vector
# GRADIENT direction toward the target is still contributed at full strength
# once reach does win the comparison). Once any safety predicate's
# robustness drops below -REACH_URGENCY_CAP, it becomes the new worst-case
# term and takes over the motion direction instead.
# NOTE: this is a robustness VALUE in metres, not a velocity, so it is NOT
# multiplied by SPEED_BOOST. The 0.5 coefficient was raised to 1.5 because at
# boosted speeds one control tick covers ~0.02 m: with the old cap, collision
# avoidance only took over the motion direction AFTER a defender had already
# stepped a full tick inside the keep-out. 1.5 gives ~3 ticks of warning.
REACH_URGENCY_CAP = 1.5 * min(COLLISION_RADIUS, DEF_SAFE_RADIUS, INTR_KEEP_OUT)   # = 0.06

MAX_STEPS = 12000                       # capped for a ~125s real flight test (see docstring note); .m file uses 10000

# ---- Intruder complexity parameters ----
NUM_WAYPOINTS = 4                      # count -> not scaled
WAYPOINT_REPLAN = 60                   # step count -> not scaled
WAYPOINT_ARRIVAL_DIST = 1.5 * SCALE
THREAT_ZONE = 6.0 * SCALE
SPRINT_ZONE = 10.0 * SCALE
DECEPTION_DURATION = 15                # step count -> not scaled
DECEPTION_COOLDOWN = 80                # step count -> not scaled
DECEPTION_PROB = 0.015                 # per-step probability -> not scaled
DECEPTION_STRENGTH = 0.7               # dimensionless blend -> not scaled
WP_STL_LAMBDA = 0.6                    # dimensionless weight between two like-scaled distances -> not scaled

# ---- Blocked-ahead forced re-route ----
# Cooldown between forced (blocked-ahead) waypoint re-routes, so an intruder
# sitting nose-to-nose with a defender re-plans its detour every few steps
# instead of every single step (which would thrash the waypoint index).
# Step-count -> NOT scaled.
FORCE_REPLAN_GAP = 8

# Flank roles: cycle through HIGH / CENTRE / LOW; Y-offset in metres -> scaled
FLANK_LABELS = ['HIGH', 'CENTRE', 'LOW', 'HIGH', 'CENTRE']
FLANK_OFFSET = np.array([8.0, 0.0, -8.0, 5.0, -5.0]) * SCALE

# ---- Defender anti-oscillation state tuning ----
MOMENTUM_ALPHA = 0.10                  # dimensionless blend -> not scaled
# switch_thresh is used as a single-cell cost DISCOUNT applied to a
# defender's previous-target column before solving Hungarian once (see the
# sticky-cost assignment logic below). The discount is subtracted from a
# cost built out of robustness values that are distances -> scale like a
# distance.
SWITCH_THRESH = 0.35 * SCALE
INTERCEPT_SMOOTH = 0.65                # dimensionless blend -> not scaled
ARRIVAL_RADIUS = BLOCK_RADIUS * 0.45

# Arena boundary keep-out margin (metres) -> scaled. Reused everywhere the
# original .m file used the literal "0.5" (defender/intruder/waypoint
# position clamps all mean the same "half a metre from the wall").
BOUNDARY_MARGIN = 0.5 * SCALE

# Role-specific offsets (distances -> scaled)
GOALKEEPER_WALL_MARGIN = 1.0 * SCALE   # goalkeeper holds at ARENA_X_MAX - this margin
CHASER_BEHIND_OFFSET = 2.0 * SCALE     # chaser role -- now actually assigned (see assign_defender_roles())

# BLOCKED (stalled) detection
BLOCKED_CHECK_START_STEP = 50          # step count -> not scaled
BLOCKED_MOVE_THRESH = 0.002 * SCALE * SPEED_BOOST   # -> m/step; boosted, else the crawl
                                       #    speed sits 60x above it and nothing is ever BLOCKED

# Numerical-safety floors used to avoid divide-by-zero.
# EPS_PHYS guards quantities that are physical distances or speeds (m or
# m/step) -> scaled like any other distance. (Collapses the .m file's several
# slightly different epsilon literals -- 0.01, 1e-3 -- into one, matching the
# simplification v2 already made.)
EPS_PHYS = 0.01 * SCALE
# EPS_UNIT guards the MAGNITUDE OF A WEIGHTED SUM OF (near-)UNIT VECTORS,
# which is dimensionless -> NOT scaled.
EPS_UNIT = 0.01

# ===================== Hardware / flight-tick constants =====================
DT = 0.05                              # matches the .m file's implicit 1-step-per-tick convention (20 Hz)

# Hard physical ceiling on how far ANY drone may be commanded to move in one
# control tick. This is a hardware limit, not a tuning knob for the game:
# grad_evade and grad_peer are absolute-magnitude pushes (NOT unit vectors
# scaled by a speed), so at high overlap their sum demands far more than one
# tick of travel -- two nearly-coincident intruders produce a grad_peer of
# ~0.9 m/step (18 m/s). That was harmless in the unboosted port only because
# every magnitude was ~31x smaller. The clamp preserves the commanded
# DIRECTION and only limits its magnitude.
MAX_SPEED_MPS = 0.5                    # m/s -- keep at or below what the CF can track
MAX_STEP_DIST = MAX_SPEED_MPS * DT     # = 0.025 m of travel per tick

# ---- Return-to-home phase (runs once the game has resolved) ----
# Which position counts as a defender's "original position":
#   False -> DEF_POS_INIT, the game's start formation (the defender's original
#            position *in the game*, i.e. where INIT_MOVE_DURATION put it).
#   True  -> cf.initialPosition, the physical launch pad from crazyflies.yaml,
#            so each defender lands back on the pad it took off from.
RETURN_TO_LAUNCH_PAD = False
RETURN_ARRIVAL_DIST = 0.01             # m -- per-defender "close enough to home" tolerance
RETURN_BRAKE_DIST = 0.30               # m -- start decelerating this far out so it settles
# Floor on the braking ramp. Without it the approach is asymptotic (speed
# falls off proportionally to the remaining distance) and the defender never
# actually closes the last centimetre.
RETURN_MIN_SPEED_FRAC = 0.15           # fraction of DEFENDER_SPEED
# Return-leg collision avoidance. Home targets can be scattered across the
# arena (notably with RETURN_TO_LAUNCH_PAD), so return paths routinely cross.
# A soft potential alone is not enough -- measured 0.07 m closest approach
# with only an overlap^2 push -- so the loop below combines an earlier,
# stronger LINEAR repulsion with a hard veto that strips out any remaining
# velocity component pointing at a neighbour already inside RETURN_MIN_SEP.
RETURN_REPULSE_RANGE = 1.5 * COLLISION_RADIUS   # = 0.30 m, where the push switches on
RETURN_REPULSE_GAIN = 3.0                       # x DEFENDER_SPEED at full overlap
RETURN_MIN_SEP = COLLISION_RADIUS               # = 0.20 m hard separation floor
RETURN_MAX_STEPS = 800                 # ticks (800 * 0.05 s = 40 s) safety cap on the return leg
RETURN_SETTLE_TIME = 1.0               # s -- let the last goTo converge before land()
CRUISE_ALTITUDE = 1.0                  # single shared altitude for both teams (m)
INIT_MOVE_DURATION = 3.0               # time to move from post-takeoff hover to game start positions (s)
TAKEOFF_HEIGHT = 0.3
TAKEOFF_DURATION = 2.0
LAND_DURATION = 5.0
LAND_TARGET_HEIGHT = 0.03
GOTO_YAW = 0.0

DEF_LABELS = [f'D{i + 1}' for i in range(NUM_DEFENDERS)]
INTRUDER_LABELS = [f'Intruder {i + 1}' for i in range(NUM_INTRUDERS)]
ROLE_STR = {1: 'Goalkeeper', 2: 'Interceptor', 3: 'Chaser'}

# ===================== LED colors =====================
# This rig's Color LED deck is driven by a single packed param,
# colorLedBot.wrgb8888 (0xWWRRGGBB) - see set_led_color() below and
# fly_show.py's identical helper pattern (set_param.py / color_led.py in
# that same folder document the param itself). Set False to skip LED
# control entirely.
ENABLE_LEDS = True
LED_DEFENDER = '#0000FF'      # blue, set once after takeoff
LED_INTRUDER = "#FF00DD"      # Magenta, set once after takeoff (default/active state)
LED_REACHED = "#37FF00"       # green, set when an intruder reaches its goal
LED_NEUTRALIZED = '#FF0000'   # red, set when an intruder is BLOCKED


# ===================== Helper functions =====================

def hex_to_rgb8(hex_color):
    """'#rrggbb' -> (r, g, b) as 0-255 ints."""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))


def set_led_color(cf, hex_color):
    """
    Bottom Color LED deck control via the single packed firmware param
    colorLedBot.wrgb8888 (uint32, 0xWWRRGGBB) - this rig has no ring.effect/
    ring.solidRed-style params. W channel left at 0 so pure RGB mixes are
    used. Wrapped in try/except and gated by ENABLE_LEDS since LEDs are
    cosmetic and should never abort the mission.
    """
    if not ENABLE_LEDS:
        return
    try:
        r, g, b = hex_to_rgb8(hex_color)
        wrgb8888 = (r << 16) | (g << 8) | b
        cf.setParam('colorLedBot.wrgb8888', wrgb8888)
    except Exception as exc:  # noqa: BLE001 - LEDs are cosmetic, never abort the mission for this
        print(f'  (LED set failed for {cf.prefix}: {exc})')

def soft_max(x, k):
    """Smooth (log-sum-exp) approximation of max(x), sharpness k."""
    x = np.asarray(x, dtype=float)
    mx = np.max(k * x)
    return mx / k + math.log(np.sum(np.exp(k * x - mx))) / k


def soft_min(x, k):
    """Smooth approximation of min(x), sharpness k."""
    return -soft_max(-np.asarray(x, dtype=float), k)


def clamp_to_arena(pos):
    """Clamp a 2D position to stay BOUNDARY_MARGIN inside the arena walls.

    The arena is centred on the origin (ARENA_X_MIN/MAX, ARENA_Y_MIN/MAX),
    not cornered at (0, 0).
    """
    return np.array([
        max(ARENA_X_MIN + BOUNDARY_MARGIN, min(ARENA_X_MAX - BOUNDARY_MARGIN, pos[0])),
        max(ARENA_Y_MIN + BOUNDARY_MARGIN, min(ARENA_Y_MAX - BOUNDARY_MARGIN, pos[1])),
    ])


def build_waypoints(start_pos, goal_pos, n_wp, flank_off, arena_w, arena_h,
                     def_pos, lam, block_rad):
    """Build a sequence of n_wp intermediate waypoints from start_pos to
    goal_pos that arc around the defender cluster using flank_off, scored
    by STL: score = -dist_to_goal - lambda * defender_proximity_penalty.

    Returns an (n_wp + 1) x 2 array; the last row is always goal_pos.
    """
    start_pos = np.asarray(start_pos, dtype=float)
    goal_pos = np.asarray(goal_pos, dtype=float)
    def_pos = np.asarray(def_pos, dtype=float)

    xs_full = np.linspace(start_pos[0], goal_pos[0], n_wp + 2)
    xs = xs_full[1:-1]                 # intermediate X positions (n_wp values)

    n_cand = 7                         # candidates per slice
    y_mid = (start_pos[1] + goal_pos[1]) / 2.0 + flank_off
    y_span = arena_h * 0.55

    wp = np.zeros((n_wp, 2))

    for k in range(n_wp):
        x_k = xs[k]
        y_cands = np.linspace(
            max(-arena_h / 2.0 + BOUNDARY_MARGIN, y_mid - y_span / 2.0),
            min(arena_h / 2.0 - BOUNDARY_MARGIN, y_mid + y_span / 2.0),
            n_cand,
        )

        best_score = -math.inf
        best_y = y_mid
        for yc in y_cands:
            cand = np.array([x_k, yc])
            to_goal = np.linalg.norm(goal_pos - cand)
            dists_to_def = np.linalg.norm(def_pos - cand, axis=1)
            min_def_dist = np.min(dists_to_def)
            def_penalty = max(0.0, block_rad * 3.0 - min_def_dist)
            score = -to_goal - lam * def_penalty
            if score > best_score:
                best_score = score
                best_y = yc

        wp[k] = [x_k, best_y]
        # NOTE: the "0.15" below is a dimensionless blend coefficient taken
        # verbatim from the original MATLAB line `yMid = bestY + flankOff*0.15;`
        # and must NOT be replaced by SCALE (this is coincidence in v2 only;
        # here SCALE=0.04 != 0.15, so keep this literal untouched).
        y_mid = best_y + flank_off * 0.15

    return np.vstack([wp, goal_pos])


def hungarian_assign(cost_matrix):
    """Optimal (Hungarian) assignment; substitutes MATLAB's matchpairs().

    cost_matrix may contain inf entries for infeasible (inactive-intruder)
    pairs -- these are replaced with a large finite number before solving,
    matching the effect of MATLAB's matchpairs unmatched-cost parameter.
    Returns an (n, 2) array of [row, col] index pairs (0-based).
    """
    cost_matrix = np.asarray(cost_matrix, dtype=float)
    safe_cost = np.where(np.isinf(cost_matrix), 1e9, cost_matrix)
    row_ind, col_ind = linear_sum_assignment(safe_cost)
    return np.stack([row_ind, col_ind], axis=1)


def assign_defender_roles(detection_accuracy):
    """Rank defenders by detection accuracy and assign Goalkeeper (1) /
    Interceptor (2) / Chaser (3) roles.

    Direct 1:1 translation of the .m file's:
        [~, accRank]  = sort(detectionAccuracy,'descend');
        goalkeeper    = accRank(end);
        interceptors  = accRank([1, 2, 4]);
        chasers       = accRank(3);
    `detection_accuracy` has exactly NUM_DEFENDERS entries here (unlike v2's
    3-defender port), so -- unlike v2's assign_defender_roles() -- no
    MATLAB index-growth quirk needs reproducing.

    Returns (def_roles, interceptor_idx) where def_roles has length
    NUM_DEFENDERS (values 1/2/3) and interceptor_idx holds the (0-based)
    indices assigned the Interceptor role, in the .m file's accRank([1,2,4])
    order (used later to pick each interceptor's pincer slot).
    """
    acc_rank = np.argsort(-detection_accuracy)     # descending-accuracy order, 0-based
    goalkeeper_idx = acc_rank[-1:]                 # accRank(end)      -> last-ranked (lowest accuracy)
    interceptor_idx = acc_rank[[0, 1, 3]]          # accRank([1,2,4])  -> 0-based [0,1,3]
    chaser_idx = acc_rank[[2]]                     # accRank(3)        -> 0-based [2]

    def_roles = np.zeros(len(detection_accuracy), dtype=int)
    def_roles[goalkeeper_idx] = 1
    def_roles[interceptor_idx] = 2
    def_roles[chaser_idx] = 3

    return def_roles, interceptor_idx


def _defender_stl_gradient(di, pos_di, target_pt, ii, role, w, cur_def,
                            cur_intr, intruder_active):
    """Analytic gradient of defender di's smooth-min STL robustness.

    Builds the conjunctive specification
        phi_di = phi_reach AND phi_sep AND phi_collision AND phi_arena AND phi_avoidIntr
    as one list of scalar predicate robustness values (rho_vals) and their
    matching gradient vectors (grad_vals), then differentiates
    soft_min(rho_vals, SOFT_K) through the softmin "attention" weights
    attn_w_i = exp(-SOFT_K*(rho_i - min(rho))) / sum_j(...), i.e. a weighted
    sum of per-predicate gradients that automatically concentrates on
    whichever predicate is closest to being violated.

    Returns (grad_dir, dn): grad_dir is the unit motion direction, dn is the
    (uncapped) distance from pos_di to target_pt (needed by the caller for
    the braking-speed profile).
    """
    vec_to_target = target_pt - pos_di
    dn = np.linalg.norm(vec_to_target)
    dir_to_target = vec_to_target / dn if dn > EPS_PHYS else np.zeros(2)

    # rho_reach_forAttn: capped VALUE used only for the attention weighting.
    # grad_reach: true unit-vector direction, uncapped -- contributed at full
    # strength whenever reach does win the comparison.
    rho_reach_forAttn = -min(dn, REACH_URGENCY_CAP)
    grad_reach = dir_to_target

    rho_vals = [w[0] * rho_reach_forAttn]
    grad_vals = [w[0] * grad_reach]

    # phi_sep (soft, role-weighted) + phi_collision (hard, unweighted)
    # against every OTHER defender.
    for dj in range(NUM_DEFENDERS):
        if dj == di:
            continue
        sep = pos_di - cur_def[dj]
        sd = max(np.linalg.norm(sep), EPS_PHYS)
        sep_dir = sep / sd
        rho_sep_j = sd - DEF_SAFE_RADIUS
        rho_coll_j = sd - COLLISION_RADIUS
        rho_vals.append(w[1] * rho_sep_j)
        grad_vals.append(w[1] * sep_dir)
        rho_vals.append(rho_coll_j)
        grad_vals.append(sep_dir)

    # phi_avoidIntr (hard, unweighted): keep clear of every INACTIVE
    # (blocked/reached) intruder that isn't this defender's own target --
    # these are frozen obstacles, not part of the assignment.
    for oi in range(NUM_INTRUDERS):
        if oi == ii or intruder_active[oi]:
            continue
        sep_i = pos_di - cur_intr[oi]
        sd_i = max(np.linalg.norm(sep_i), EPS_PHYS)
        sep_dir_i = sep_i / sd_i
        rho_avoid = sd_i - INTR_KEEP_OUT
        rho_vals.append(rho_avoid)
        grad_vals.append(sep_dir_i)

    # phi_arena (soft, role-weighted): distance to each of the 4 walls.
    margins = np.array([
        pos_di[0] - (ARENA_X_MIN + BOUNDARY_MARGIN),
        (ARENA_X_MAX - BOUNDARY_MARGIN) - pos_di[0],
        pos_di[1] - (ARENA_Y_MIN + BOUNDARY_MARGIN),
        (ARENA_Y_MAX - BOUNDARY_MARGIN) - pos_di[1],
    ])
    margin_grads = np.array([[1.0, -1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -1.0]])
    for k in range(4):
        rho_vals.append(w[2] * margins[k])
        grad_vals.append(w[2] * margin_grads[:, k])

    rho_vals = np.array(rho_vals)
    grad_vals = np.array(grad_vals)   # shape (N, 2)

    attn_w = np.exp(-SOFT_K * (rho_vals - np.min(rho_vals)))
    attn_w = attn_w / np.sum(attn_w)
    grad_stl = grad_vals.T @ attn_w   # shape (2,)

    gn = np.linalg.norm(grad_stl)
    grad_dir = dir_to_target if gn < EPS_UNIT else grad_stl / gn
    return grad_dir, dn


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    by_name = swarm.allcfs.crazyfliesByName
    missing = [n for n in DEFENDER_CF_NAMES + INTRUDER_CF_NAMES if n not in by_name]
    if missing:
        raise RuntimeError(f'Crazyflies not found in config: {missing}')

    def_cfs = [by_name[n] for n in DEFENDER_CF_NAMES]
    intr_cfs = [by_name[n] for n in INTRUDER_CF_NAMES]

    rng = np.random.default_rng()

    # ===================== Role Assignment =====================
    def_roles, interceptor_idx = assign_defender_roles(DETECTION_ACCURACY)
    interceptor_slot = {di: slot for slot, di in enumerate(interceptor_idx)}

    print('===== STL Game - Complex Intruders - Role Assignment =====')
    for di in range(NUM_DEFENDERS):
        print(f'  {DEF_LABELS[di]} (acc={DETECTION_ACCURACY[di]:.2f}): {ROLE_STR[def_roles[di]]}')
    print('\nIntruder Flank Assignments:')
    for ii in range(NUM_INTRUDERS):
        print(f'  I{ii + 1}: {FLANK_LABELS[ii]} (Y-offset={FLANK_OFFSET[ii]:.2f})')

    # ===================== State =====================
    cur_def = DEF_POS_INIT.copy()
    cur_intr = INTRUDER_POS_INIT.copy()

    intruder_active = np.ones(NUM_INTRUDERS, dtype=bool)
    intruder_blocked = np.zeros(NUM_INTRUDERS, dtype=bool)
    intruder_landed = np.zeros(NUM_INTRUDERS, dtype=bool)   # True once land() has been issued for this intruder
    intruder_vel = np.zeros((NUM_INTRUDERS, 2))

    def_vel = np.zeros((NUM_DEFENDERS, 2))
    prev_def_targets = np.full(NUM_DEFENDERS, -1, dtype=int)
    smoothed_intercept = np.zeros((NUM_DEFENDERS, 2))

    # Waypoints / re-plan bookkeeping
    intruder_waypoints = [None] * NUM_INTRUDERS
    intruder_wp_idx = np.zeros(NUM_INTRUDERS, dtype=int)
    intruder_last_replan = np.full(NUM_INTRUDERS, -WAYPOINT_REPLAN, dtype=int)
    intruder_force_cooldown = np.zeros(NUM_INTRUDERS, dtype=int)

    # Deception state
    deception_active = np.zeros(NUM_INTRUDERS, dtype=bool)
    deception_timer = np.zeros(NUM_INTRUDERS, dtype=int)
    deception_dir = np.zeros((NUM_INTRUDERS, 2))
    deception_cool_timer = np.zeros(NUM_INTRUDERS, dtype=int)

    # Lightweight per-intruder summary accumulators (replace full
    # trajectory-logging arrays, since no plotting is required)
    total_dist = np.zeros(NUM_INTRUDERS)
    speed_sum = np.zeros(NUM_INTRUDERS)
    speed_count = np.zeros(NUM_INTRUDERS, dtype=int)

    for ii in range(NUM_INTRUDERS):
        intruder_waypoints[ii] = build_waypoints(
            INTRUDER_POS_INIT[ii], INTRUDER_GOALS[ii], NUM_WAYPOINTS,
            FLANK_OFFSET[ii], ARENA_W, ARENA_H, DEF_POS_INIT, WP_STL_LAMBDA,
            BLOCK_RADIUS,
        )

    # ===================== Arm / takeoff / move to start =====================
    mission_cfs = def_cfs + intr_cfs
    for cf in mission_cfs:
        cf.arm(True)
    timeHelper.sleep(1.0)

    for cf in mission_cfs:
        cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    for di, cf in enumerate(def_cfs):
        goal = [cur_def[di, 0], cur_def[di, 1], CRUISE_ALTITUDE]
        cf.goTo(goal, GOTO_YAW, INIT_MOVE_DURATION)
    for ii, cf in enumerate(intr_cfs):
        goal = [cur_intr[ii, 0], cur_intr[ii, 1], CRUISE_ALTITUDE]
        cf.goTo(goal, GOTO_YAW, INIT_MOVE_DURATION)
    timeHelper.sleep(INIT_MOVE_DURATION)

    # Set each team's base LED color for the mission (no-op unless
    # ENABLE_LEDS is True - see set_led_color()). Per-intruder color
    # changes to LED_REACHED/LED_NEUTRALIZED happen later, at the
    # REACHED/BLOCKED detection points below.
    for cf in def_cfs:
        set_led_color(cf, LED_DEFENDER)
    for cf in intr_cfs:
        set_led_color(cf, LED_INTRUDER)

    # ===================== Simulation / flight loop =====================
    final_step = MAX_STEPS

    for step in range(MAX_STEPS - 1):
        curDef = cur_def
        curIntr = cur_intr

        # --- STL-based Assignment ---
        active_idx = np.where(intruder_active)[0]
        cost_matrix = np.full((NUM_DEFENDERS, NUM_INTRUDERS), np.inf)

        # ---- Urgency score per active intruder ----
        urgency = np.zeros(NUM_INTRUDERS)
        for ii in active_idx:
            dist_goal = np.linalg.norm(curIntr[ii] - INTRUDER_GOALS[ii])
            progress_frac = max(0.0, 1.0 - dist_goal / max(INIT_GOAL_DIST[ii], EPS_PHYS))
            speed_frac = min(1.0, np.linalg.norm(intruder_vel[ii]) / INTRUDER_SPEED_MAX)
            urgency[ii] = URGENCY_GOAL_W * progress_frac + URGENCY_SPEED_W * speed_frac

        for di in range(NUM_DEFENDERS):
            for ii in active_idx:
                vel_ii = intruder_vel[ii]
                goal_dir_ii = INTRUDER_GOALS[ii] - curIntr[ii]
                gd = np.linalg.norm(goal_dir_ii)
                if gd > 0:
                    goal_dir_ii = goal_dir_ii / gd
                rho_block_horizon = np.zeros(STL_HORIZON)
                for h in range(1, STL_HORIZON + 1):
                    predI = curIntr[ii] + h * (INTRUDER_SPEED * goal_dir_ii + vel_ii * 0.5)
                    predI = clamp_to_arena(predI)
                    dv = predI - curDef[di]
                    dvn = np.linalg.norm(dv)
                    if dvn > 0:
                        dv = dv / dvn
                    predD = curDef[di] + min(h * DEFENDER_SPEED, dvn) * dv
                    rho_block_horizon[h - 1] = BLOCK_RADIUS - np.linalg.norm(predD - predI)
                rho_F = soft_max(rho_block_horizon, SOFT_K)
                cost_matrix[di, ii] = -rho_F * (1.0 + URGENCY_GAIN * urgency[ii])

        def_targets = np.zeros(NUM_DEFENDERS, dtype=int)
        if active_idx.size > 0:
            # ---- Continuity bias (anti-oscillation), applied BEFORE solving ----
            sticky_cost = cost_matrix.copy()
            for di in range(NUM_DEFENDERS):
                pt = prev_def_targets[di]
                if pt >= 0 and intruder_active[pt] and np.isfinite(sticky_cost[di, pt]):
                    sticky_cost[di, pt] -= SWITCH_THRESH

            assignment = hungarian_assign(sticky_cost)
            assigned = np.zeros(NUM_DEFENDERS, dtype=bool)
            for di2, ii2 in assignment:
                if ii2 < NUM_INTRUDERS and intruder_active[ii2]:
                    def_targets[di2] = ii2
                    assigned[di2] = True
            for di in range(NUM_DEFENDERS):
                if not assigned[di]:
                    def_targets[di] = int(np.argmin(cost_matrix[di, :]))
        else:
            def_targets[:] = 0
        prev_def_targets = def_targets.copy()

        # --- Defender STL gradient control (TRUE STL-gradient controller) ---
        new_def = curDef.copy()
        for di in range(NUM_DEFENDERS):
            ii = def_targets[di]
            pos_di = curDef[di]
            pos_ii = curIntr[ii]
            goal_ii = INTRUDER_GOALS[ii]
            vel_ii = intruder_vel[ii]
            role = def_roles[di]
            w = ROLE_WEIGHTS[role - 1]

            dist_def = np.linalg.norm(pos_di - pos_ii)
            t_est = dist_def / max(DEFENDER_SPEED, EPS_PHYS)
            predI = pos_ii + min(t_est, LOOKAHEAD) * vel_ii
            predI = clamp_to_arena(predI)
            path_vec = goal_ii - pos_ii
            path_len = np.linalg.norm(path_vec)
            path_dir = path_vec / path_len if path_len > 0 else np.zeros(2)
            t_proj = np.dot(predI - pos_ii, path_dir)
            t_proj = max(0.0, min(path_len, t_proj))
            raw_intercept = pos_ii + t_proj * path_dir
            raw_intercept = raw_intercept * (1 - INTERCEPT_BIAS) + goal_ii * INTERCEPT_BIAS
            perp_dir = np.array([-path_dir[1], path_dir[0]])

            if role == 1:       # Goalkeeper
                raw_intercept = np.array([ARENA_X_MAX - GOALKEEPER_WALL_MARGIN, pos_ii[1]])
            elif role == 2:     # Interceptor
                slot = interceptor_slot.get(di, 0)
                offsets = np.array([-PINCER_SPREAD, 0.0, PINCER_SPREAD])
                raw_intercept = raw_intercept + offsets[min(slot, 2)] * perp_dir
            elif role == 3:     # Chaser (now actually assigned -- see assign_defender_roles())
                raw_intercept = pos_ii - CHASER_BEHIND_OFFSET * path_dir

            if step == 0 or np.all(smoothed_intercept[di] == 0):
                smoothed_intercept[di] = raw_intercept
            else:
                smoothed_intercept[di] = (INTERCEPT_SMOOTH * raw_intercept
                                           + (1 - INTERCEPT_SMOOTH) * smoothed_intercept[di])
            intercept_pt = smoothed_intercept[di]
            dist_to_intercept = np.linalg.norm(intercept_pt - pos_di)
            dist_to_intruder = np.linalg.norm(pos_ii - pos_di)
            if dist_to_intercept < ARRIVAL_RADIUS or dist_to_intruder < BLOCK_RADIUS * 1.1:
                target_pt = pos_ii
            else:
                target_pt = intercept_pt

            grad_dir, dn = _defender_stl_gradient(
                di, pos_di, target_pt, ii, role, w, curDef, curIntr, intruder_active,
            )

            # _2_5_4 keeps the tightened braking profile from the previous
            # revision: brake_dist shrunk (3.0 -> 1.5 * BLOCK_RADIUS) so
            # defenders stay at full speed longer, and the speed-scale floor
            # raised (0.2 -> 0.5) so they never crawl near the intruder --
            # always closing in with purpose.
            brake_dist = 1.5 * BLOCK_RADIUS
            speed_scale = min(1.0, max(0.5, dn / brake_dist))
            raw_vel = DEFENDER_SPEED * speed_scale * grad_dir
            if dist_to_intercept < ARRIVAL_RADIUS:
                def_vel[di] = 0.0
            blended_vel = MOMENTUM_ALPHA * def_vel[di] + (1 - MOMENTUM_ALPHA) * raw_vel
            def_vel[di] = blended_vel

            new_def[di] = clamp_to_arena(curDef[di] + blended_vel)

        # --- Complex Intruder Control ---
        new_intr_all = np.zeros((NUM_INTRUDERS, 2))
        for ii in range(NUM_INTRUDERS):
            if not intruder_active[ii]:
                new_intr_all[ii] = curIntr[ii]
                continue

            pos_ii = curIntr[ii]
            goal_ii = INTRUDER_GOALS[ii]

            # Hard block check
            blocked = False
            for di in range(NUM_DEFENDERS):
                if np.linalg.norm(pos_ii - new_def[di]) <= BLOCK_RADIUS:
                    blocked = True
                    break
            if blocked:
                new_intr_all[ii] = pos_ii
                intruder_vel[ii] = 0.0
                continue

            if intruder_force_cooldown[ii] > 0:
                intruder_force_cooldown[ii] -= 1

            # ---- Blocked-ahead detection ----
            goal_dir_now = (goal_ii - pos_ii) / max(np.linalg.norm(goal_ii - pos_ii), EPS_PHYS)
            blocked_ahead = False
            for di in range(NUM_DEFENDERS):
                dvec_check = new_def[di] - pos_ii
                dd_check = np.linalg.norm(dvec_check)
                if dd_check < REPULSION_RANGE and np.dot(dvec_check, goal_dir_now) > 0:
                    blocked_ahead = True
                    break
            if blocked_ahead and intruder_force_cooldown[ii] <= 0:
                intruder_last_replan[ii] = step - WAYPOINT_REPLAN   # trigger the re-plan check below this step
                intruder_force_cooldown[ii] = FORCE_REPLAN_GAP

            # --- Layer 1: Waypoint re-planning ---
            wp = intruder_waypoints[ii]
            wpi = intruder_wp_idx[ii]

            if (step - intruder_last_replan[ii]) >= WAYPOINT_REPLAN:
                wp = build_waypoints(
                    pos_ii, INTRUDER_GOALS[ii], NUM_WAYPOINTS, FLANK_OFFSET[ii],
                    ARENA_W, ARENA_H, new_def, WP_STL_LAMBDA, BLOCK_RADIUS,
                )
                intruder_waypoints[ii] = wp
                intruder_last_replan[ii] = step
                wpi = 0
                intruder_wp_idx[ii] = wpi

            while wpi < len(wp) - 1 and np.linalg.norm(pos_ii - wp[wpi]) < WAYPOINT_ARRIVAL_DIST:
                wpi += 1
            wpi = min(wpi, len(wp) - 1)
            intruder_wp_idx[ii] = wpi
            current_target = wp[wpi]

            # --- Layer 2: Deception / feinting ---
            if deception_cool_timer[ii] > 0:
                deception_cool_timer[ii] -= 1

            if deception_active[ii]:
                deception_timer[ii] -= 1
                if deception_timer[ii] <= 0:
                    deception_active[ii] = False
                    deception_cool_timer[ii] = DECEPTION_COOLDOWN
            else:
                min_def_dist_ii = np.min(np.linalg.norm(new_def - curIntr[ii], axis=1))
                if (deception_cool_timer[ii] == 0 and min_def_dist_ii < THREAT_ZONE * 1.5
                        and rng.random() < DECEPTION_PROB):
                    deception_active[ii] = True
                    deception_timer[ii] = DECEPTION_DURATION
                    to_goal = (goal_ii - pos_ii) / max(np.linalg.norm(goal_ii - pos_ii), EPS_PHYS)
                    perp_to_goal = np.array([-to_goal[1], to_goal[0]])
                    def_centroid = np.mean(new_def, axis=0)
                    side = np.sign(np.dot(perp_to_goal, pos_ii - def_centroid))
                    if side == 0:
                        side = 1
                    deception_dir[ii] = side * perp_to_goal

            # --- Layer 3: Primary direction toward current waypoint ---
            to_wp = current_target - pos_ii
            to_wp_dist = np.linalg.norm(to_wp)
            if to_wp_dist > EPS_PHYS:
                grad_reach = to_wp / to_wp_dist
            else:
                grad_reach = (goal_ii - pos_ii) / max(np.linalg.norm(goal_ii - pos_ii), EPS_PHYS)

            if deception_active[ii]:
                frac = deception_timer[ii] / DECEPTION_DURATION
                grad_reach = ((1 - frac * DECEPTION_STRENGTH) * grad_reach
                              + frac * DECEPTION_STRENGTH * deception_dir[ii])
                grad_reach = grad_reach / max(np.linalg.norm(grad_reach), EPS_UNIT)

            # --- Layer 4: Defender evade (additive, not normalised away) ---
            grad_evade = np.zeros(2)
            perp_goal = np.array([-grad_reach[1], grad_reach[0]])
            for di in range(NUM_DEFENDERS):
                dvec = pos_ii - new_def[di]
                dd = np.linalg.norm(dvec)
                if dd < REPULSION_RANGE and dd > EPS_PHYS:
                    overlap = (REPULSION_RANGE - dd) / REPULSION_RANGE
                    unit_away = dvec / dd
                    # Tangential-only repulsion: if this defender is roughly
                    # ahead of the intruder along its goal direction (i.e. it
                    # is the one actually blocking the path), strip out the
                    # backward-pointing component of the radial push, so a
                    # head-on blocker's repulsion can't overpower grad_reach
                    # and send the intruder straight backward.
                    ahead_dot = np.dot(new_def[di] - pos_ii, grad_reach)
                    par_amt = np.dot(unit_away, grad_reach)
                    if ahead_dot > 0 and par_amt < 0:
                        unit_away = unit_away - par_amt * grad_reach
                        nrm = np.linalg.norm(unit_away)
                        unit_away = unit_away / nrm if nrm > EPS_UNIT else perp_goal
                    radial = REPULSION_STRENGTH * overlap ** 2 * unit_away
                    side = np.sign(np.dot(dvec, perp_goal))
                    channel = CHANNEL_STRENGTH * overlap * side * perp_goal
                    grad_evade = grad_evade + radial + channel

            # Intruder-intruder repulsion
            grad_peer = np.zeros(2)
            for oi in range(NUM_INTRUDERS):
                if oi != ii:
                    o_pos = curIntr[oi]
                    iv = pos_ii - o_pos
                    dd = np.linalg.norm(iv)
                    if dd < INTR_SAFE_RADIUS * 2.0 and dd > EPS_PHYS:
                        overlap = (INTR_SAFE_RADIUS * 2.0 - dd) / (INTR_SAFE_RADIUS * 2.0)
                        grad_peer = grad_peer + INTR_REP_STRENGTH * overlap ** 2 * 6.0 * (iv / dd)

            # --- Layer 5: Threat-aware speed modulation ---
            min_def_dist_ii = np.min(np.linalg.norm(new_def - curIntr[ii], axis=1))
            if min_def_dist_ii < THREAT_ZONE:
                threat_frac = (THREAT_ZONE - min_def_dist_ii) / THREAT_ZONE
                active_speed = INTRUDER_SPEED * (1 - 0.6 * threat_frac)
                active_speed = max(active_speed, INTRUDER_SPEED_MIN)
            elif min_def_dist_ii > SPRINT_ZONE:
                active_speed = INTRUDER_SPEED_MAX
            else:
                active_speed = INTRUDER_SPEED

            step_vec = (active_speed * grad_reach
                        + 0.35 * grad_evade
                        + grad_peer)
            # Enforce the physical per-tick travel limit (see MAX_STEP_DIST).
            # Direction is preserved; only the magnitude is capped.
            step_norm = np.linalg.norm(step_vec)
            if step_norm > MAX_STEP_DIST:
                step_vec = step_vec * (MAX_STEP_DIST / step_norm)
            new_pos = clamp_to_arena(pos_ii + step_vec)
            new_intr_all[ii] = new_pos
            intruder_vel[ii] = new_pos - pos_ii

            total_dist[ii] += np.linalg.norm(new_pos - pos_ii)
            speed_sum[ii] += active_speed
            speed_count[ii] += 1

        # --- REACHED / BLOCKED detection ---
        for ii in range(NUM_INTRUDERS):
            if intruder_active[ii]:
                if np.linalg.norm(new_intr_all[ii] - INTRUDER_GOALS[ii]) < GOAL_RADIUS:
                    intruder_active[ii] = False
                    print(f'Step {step + 1}: {INTRUDER_LABELS[ii]} REACHED goal!')
                    set_led_color(intr_cfs[ii], LED_REACHED)
                elif (step + 1 > BLOCKED_CHECK_START_STEP
                        and np.linalg.norm(new_intr_all[ii] - curIntr[ii]) < BLOCKED_MOVE_THRESH):
                    intruder_active[ii] = False
                    intruder_blocked[ii] = True
                    print(f'Step {step + 1}: {INTRUDER_LABELS[ii]} BLOCKED!')
                    set_led_color(intr_cfs[ii], LED_NEUTRALIZED)
                    # Land this intruder immediately instead of waiting for
                    # the mission to end -- a BLOCKED intruder is out of the
                    # game, so free up its airspace right away.
                    intr_cfs[ii].land(targetHeight=LAND_TARGET_HEIGHT, duration=LAND_DURATION)
                    intruder_landed[ii] = True

        cur_def = new_def
        cur_intr = new_intr_all

        # --- Stream this tick's positions to hardware ---
        for di, cf in enumerate(def_cfs):
            goal = [cur_def[di, 0], cur_def[di, 1], CRUISE_ALTITUDE]
            cf.goTo(goal, GOTO_YAW, DT)
        for ii, cf in enumerate(intr_cfs):
            if intruder_landed[ii]:
                continue   # already landing/landed -- don't override its land trajectory with goTo
            goal = [cur_intr[ii, 0], cur_intr[ii, 1], CRUISE_ALTITUDE]
            cf.goTo(goal, GOTO_YAW, DT)
        timeHelper.sleep(DT)

        if not np.any(intruder_active):
            final_step = step + 2   # +1 for 0-index, +1 because this tick already advanced state
            break
    else:
        final_step = MAX_STEPS

    # ===================== Land the intruders first =====================
    # Clearing the intruders out of the air BEFORE walking the defenders home
    # means the return legs below can never cross a still-hovering intruder.
    for ii, cf in enumerate(intr_cfs):
        if not intruder_landed[ii]:
            cf.land(targetHeight=LAND_TARGET_HEIGHT, duration=LAND_DURATION)
            intruder_landed[ii] = True
    timeHelper.sleep(LAND_DURATION)

    # ===================== Defenders return to their original positions =====================
    # Streamed as discrete goTo(..., DT) ticks, exactly like the game loop, so
    # the return leg (a) obeys the same MAX_STEP_DIST speed ceiling, (b) brakes
    # into the home point instead of overshooting it, and (c) keeps defenders
    # COLLISION_RADIUS apart on the way. A single straight-line goTo per
    # defender would be simpler but lets crossing return paths collide --
    # defenders end the game scattered across the arena, so their paths back to
    # the start formation routinely cross.
    if RETURN_TO_LAUNCH_PAD:
        home_targets = np.array([np.asarray(cf.initialPosition, dtype=float)[:2]
                                 for cf in def_cfs])
        home_desc = 'launch pads'
    else:
        home_targets = DEF_POS_INIT.copy()
        home_desc = 'start formation'

    print(f'\nAll intruders resolved -- defenders returning to {home_desc}...')
    all_home = False
    return_steps = 0
    for return_steps in range(1, RETURN_MAX_STEPS + 1):
        new_def = np.zeros((NUM_DEFENDERS, 2))
        all_home = True
        for di in range(NUM_DEFENDERS):
            to_home = home_targets[di] - cur_def[di]
            dist_home = np.linalg.norm(to_home)
            if dist_home < RETURN_ARRIVAL_DIST:
                # Parked: command the EXACT home point, not the current
                # position, so the final streamed setpoint is the target and
                # the onboard controller converges onto it during
                # RETURN_SETTLE_TIME rather than freezing a few cm short.
                new_def[di] = home_targets[di]
                continue
            all_home = False
            dir_home = to_home / dist_home

            # Defender-defender separation. Linear (not squared) overlap over
            # a range wider than the floor, so the push ramps up well before
            # contact rather than only once already too close.
            push = np.zeros(2)
            for dj in range(NUM_DEFENDERS):
                if dj == di:
                    continue
                sep = cur_def[di] - cur_def[dj]
                sd = np.linalg.norm(sep)
                if EPS_PHYS < sd < RETURN_REPULSE_RANGE:
                    overlap = (RETURN_REPULSE_RANGE - sd) / RETURN_REPULSE_RANGE
                    push = push + overlap * (sep / sd)

            # Decelerate into home so the drone settles rather than buzzing
            # around the target at full speed.
            speed_scale = min(1.0, max(RETURN_MIN_SPEED_FRAC,
                                       dist_home / RETURN_BRAKE_DIST))
            # Never travel further than the remaining distance, so the
            # defender settles onto home instead of overshooting and
            # oscillating around it.
            travel = min(DEFENDER_SPEED * speed_scale, dist_home)
            step_vec = travel * dir_home + DEFENDER_SPEED * RETURN_REPULSE_GAIN * push

            # Hard veto: for any neighbour already inside RETURN_MIN_SEP,
            # remove the component of this step that points toward it, so the
            # separation can never be driven further down no matter how
            # strongly the homing term pulls.
            for dj in range(NUM_DEFENDERS):
                if dj == di:
                    continue
                sep = cur_def[di] - cur_def[dj]
                sd = np.linalg.norm(sep)
                if EPS_PHYS < sd < RETURN_MIN_SEP:
                    toward = -sep / sd                  # unit vector at the neighbour
                    closing = np.dot(step_vec, toward)
                    if closing > 0:
                        step_vec = step_vec - closing * toward

            step_norm = np.linalg.norm(step_vec)
            if step_norm > MAX_STEP_DIST:
                step_vec = step_vec * (MAX_STEP_DIST / step_norm)
            new_def[di] = clamp_to_arena(cur_def[di] + step_vec)

        cur_def = new_def
        for di, cf in enumerate(def_cfs):
            goal = [cur_def[di, 0], cur_def[di, 1], CRUISE_ALTITUDE]
            cf.goTo(goal, GOTO_YAW, DT)
        timeHelper.sleep(DT)
        if all_home:
            break

    home_err = np.linalg.norm(cur_def - home_targets, axis=1)
    print(f'Return leg: {return_steps} steps ({return_steps * DT:.1f} s) | '
          f'max home error = {np.max(home_err):.4f} m')
    if not all_home:
        print(f'  WARNING: RETURN_MAX_STEPS ({RETURN_MAX_STEPS}) reached before every '
              f'defender was home -- landing from current positions.')
    timeHelper.sleep(RETURN_SETTLE_TIME)   # let the final goTo converge

    # ===================== Land the defenders =====================
    for cf in def_cfs:
        cf.land(targetHeight=LAND_TARGET_HEIGHT, duration=LAND_DURATION)
    timeHelper.sleep(LAND_DURATION)
    for cf in mission_cfs:
        cf.arm(False)

    # ===================== Print Summary =====================
    print('\n===== Complex Intruder STL 2D Summary =====')
    print(f'Total steps: {final_step} | Arena: {ARENA_W:.2f}x{ARENA_H:.2f} m')
    print(f'Speed boost: {SPEED_BOOST:.2f}x | tick: {DT:.3f} s')
    print(f'Intruder speed: {INTRUDER_SPEED_MIN:.5f}-{INTRUDER_SPEED_MAX:.5f} m/step '
          f'({INTRUDER_SPEED_MIN / DT:.3f}-{INTRUDER_SPEED_MAX / DT:.3f} m/s) '
          f'| Defender speed: {DEFENDER_SPEED:.5f} m/step '
          f'({DEFENDER_SPEED / DT:.3f} m/s)')
    n_blocked = int(np.sum(intruder_blocked))
    n_reached = int(np.sum(~intruder_active & ~intruder_blocked))
    n_still_active = int(np.sum(intruder_active))
    print(f'Outcome: {n_blocked} BLOCKED | {n_reached} REACHED goal | {n_still_active} ACTIVE')
    for ii in range(NUM_INTRUDERS):
        final_dist = np.linalg.norm(cur_intr[ii] - INTRUDER_GOALS[ii])
        avg_spd = speed_sum[ii] / speed_count[ii] if speed_count[ii] > 0 else 0.0
        if intruder_blocked[ii]:
            status = 'BLOCKED'
        elif not intruder_active[ii]:
            status = 'REACHED'
        else:
            status = 'ACTIVE'
        print(f'  {INTRUDER_LABELS[ii]} [{FLANK_LABELS[ii]}]: {status} | '
              f'd_goal={final_dist:.4f} | dist={total_dist[ii]:.4f} | avg_spd={avg_spd:.5f}')


if __name__ == '__main__':
    main()
