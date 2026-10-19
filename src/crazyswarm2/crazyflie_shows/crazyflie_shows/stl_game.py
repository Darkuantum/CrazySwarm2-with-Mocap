#!/usr/bin/env python3
"""The STL pursuit-evasion game -- the algorithm only. No ROS, no radio.

A COPY, refactored, of a colleague's script, which stays unchanged where it
arrived (``crazyflie_examples/stl_pursuit_evasion_game_v3.py``, delivered
over Slack, not git). This module is what ``stl_show`` flies and
``plan_stl`` checks. Design and the v4 procedure: ``crazyflie_shows/STL.md`` (package root).

The game in one paragraph: 5 defenders (Goalkeeper / Interceptor / Chaser
roles from a fixed accuracy ranking) chase 5 intruders using the analytic
gradient of a smooth-min STL robustness, with a Hungarian assignment that is
reweighted by each intruder's urgency. Intruders follow re-planned waypoints
around the defenders, feint at random and modulate speed by threat. An
intruder is REACHED inside ``GOAL_RADIUS`` of its goal and BLOCKED when it
stalls (a defender within ``BLOCK_RADIUS`` freezes it); a BLOCKED intruder
lands at once. The original's docstring has the full account of every
constant; it is not repeated here.

Why a copy can be flown from a TABLE
------------------------------------
The original never reads mocap: every position it commands comes from its
own internal model, and its only randomness is one unseeded generator. So
for a given seed the whole flight -- every ``goTo``, ``land`` and LED cue, at
every tick -- is fixed before takeoff. :func:`play` computes that table and
:func:`verify` checks it; ``stl_show`` then streams the very table that was
checked. That is this package's planner contract (WRITING-A-SHOW.md
section 2) at no cost to the algorithm. A future version that closes the loop
on mocap breaks this property, and with it the pre-flight proof.

Faithfulness
------------
Constants and helpers below are the original's, line for line; only
``main()`` was restructured into :func:`play` (state in, table out) and the
hardware calls moved to ``stl_show``. Arithmetic is kept in the original's
order so the two agree to the bit: ``plan_stl --against <original.py>``
runs both on the same seeds and diffs every command they send.
"""

import hashlib
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

#: sha256 of the original this port was made from (v3, received 2026-10-13).
#: ``plan_stl --against`` reports whether a file is this version or a new one.
SOURCE_SHA256 = 'a9481630084d63a9eb478452c0b99a6e432c7976222312fc1c71e6f5771dd8ed'
SOURCE_NAME = 'stl_pursuit_evasion_game_v3.py'

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
# Bottom Color LED deck, one packed param colorLedBot.wrgb8888 (0xWWRRGGBB),
# W left at 0. The game only decides WHICH colour and WHEN (Frame.events);
# stl_show sends it. Set False to skip LED control entirely.
# NOTE LED_REACHED is green, which is also the rig's "connected, ready" colour
# (CLAUDE.md, LED convention) -- kept because it is the original's choice.
ENABLE_LEDS = True
LED_DEFENDER = '#0000FF'      # blue, set once after takeoff
LED_INTRUDER = '#FF00DD'      # Magenta, set once after takeoff (default/active state)
LED_REACHED = '#37FF00'       # green, set when an intruder reaches its goal
LED_NEUTRALIZED = '#FF0000'   # red, set when an intruder is BLOCKED

# ===================== Helper functions =====================


def hex_to_rgb8(hex_color):
    """'#rrggbb' -> (r, g, b) as 0-255 ints."""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))


def led_value(hex_color):
    """'#rrggbb' -> the colorLedBot.wrgb8888 integer the original sends."""
    r, g, b = hex_to_rgb8(hex_color)
    return (r << 16) | (g << 8) | b


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


# ===================== The game as a table =====================


@dataclass
class Frame:
    """One control tick: what the original sends, in the order it sends it.

    ``events`` come first (prints, LED cues and ``land`` calls, interleaved
    exactly as the original issues them), then one ``goTo`` per defender and
    one per intruder whose ``streaming`` flag is set, then a ``DT`` sleep.
    """

    defenders: np.ndarray          # (NUM_DEFENDERS, 2) goal xy
    intruders: np.ndarray          # (NUM_INTRUDERS, 2) goal xy
    streaming: np.ndarray          # (NUM_INTRUDERS,) bool: still sent a goTo
    events: list = field(default_factory=list)   # ('say', text) | ('led', ii, hex) | ('land', ii)


@dataclass
class Game:
    """Everything :func:`play` decided, before any drone moves."""

    seed: int
    def_roles: np.ndarray
    header: list                   # the role/flank lines the original prints
    frames: list                   # game ticks (Frame)
    returns: list                  # defender goal xy per return tick
    home_desc: str
    return_lines: list             # printed after the return leg
    summary: list                  # the original's closing summary lines
    final_step: int
    intruder_blocked: np.ndarray
    intruder_active: np.ndarray

    @property
    def outcome(self):
        """The original's ``Outcome:`` line."""
        return next(s for s in self.summary if s.startswith('Outcome:'))

    @property
    def game_s(self):
        return len(self.frames) * DT

    @property
    def return_s(self):
        return len(self.returns) * DT


def play(seed, launch_pads=None):
    """Run the whole game in its own model and return the flight as a table.

    ``main()`` of the original from ``rng = ...`` to the summary, with every
    hardware call recorded instead of sent. ``launch_pads`` (NUM_DEFENDERS x
    2, the defenders' ``initial_position`` xy) is only read when
    ``RETURN_TO_LAUNCH_PAD`` is True, as in the original.
    """
    rng = np.random.default_rng(seed)

    # ===================== Role Assignment =====================
    def_roles, interceptor_idx = assign_defender_roles(DETECTION_ACCURACY)
    interceptor_slot = {di: slot for slot, di in enumerate(interceptor_idx)}

    header = ['===== STL Game - Complex Intruders - Role Assignment =====']
    for di in range(NUM_DEFENDERS):
        header.append(f'  {DEF_LABELS[di]} (acc={DETECTION_ACCURACY[di]:.2f}): '
                      f'{ROLE_STR[def_roles[di]]}')
    header.append('\nIntruder Flank Assignments:')
    for ii in range(NUM_INTRUDERS):
        header.append(f'  I{ii + 1}: {FLANK_LABELS[ii]} (Y-offset={FLANK_OFFSET[ii]:.2f})')

    # ===================== State =====================
    cur_def = DEF_POS_INIT.copy()
    cur_intr = INTRUDER_POS_INIT.copy()

    intruder_active = np.ones(NUM_INTRUDERS, dtype=bool)
    intruder_blocked = np.zeros(NUM_INTRUDERS, dtype=bool)
    intruder_landed = np.zeros(NUM_INTRUDERS, dtype=bool)
    intruder_vel = np.zeros((NUM_INTRUDERS, 2))

    def_vel = np.zeros((NUM_DEFENDERS, 2))
    prev_def_targets = np.full(NUM_DEFENDERS, -1, dtype=int)
    smoothed_intercept = np.zeros((NUM_DEFENDERS, 2))

    intruder_waypoints = [None] * NUM_INTRUDERS
    intruder_wp_idx = np.zeros(NUM_INTRUDERS, dtype=int)
    intruder_last_replan = np.full(NUM_INTRUDERS, -WAYPOINT_REPLAN, dtype=int)
    intruder_force_cooldown = np.zeros(NUM_INTRUDERS, dtype=int)

    deception_active = np.zeros(NUM_INTRUDERS, dtype=bool)
    deception_timer = np.zeros(NUM_INTRUDERS, dtype=int)
    deception_dir = np.zeros((NUM_INTRUDERS, 2))
    deception_cool_timer = np.zeros(NUM_INTRUDERS, dtype=int)

    total_dist = np.zeros(NUM_INTRUDERS)
    speed_sum = np.zeros(NUM_INTRUDERS)
    speed_count = np.zeros(NUM_INTRUDERS, dtype=int)

    for ii in range(NUM_INTRUDERS):
        intruder_waypoints[ii] = build_waypoints(
            INTRUDER_POS_INIT[ii], INTRUDER_GOALS[ii], NUM_WAYPOINTS,
            FLANK_OFFSET[ii], ARENA_W, ARENA_H, DEF_POS_INIT, WP_STL_LAMBDA,
            BLOCK_RADIUS,
        )

    frames = []
    final_step = MAX_STEPS

    for step in range(MAX_STEPS - 1):
        events = []
        curDef = cur_def
        curIntr = cur_intr

        # --- STL-based Assignment ---
        active_idx = np.where(intruder_active)[0]
        cost_matrix = np.full((NUM_DEFENDERS, NUM_INTRUDERS), np.inf)

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

        # --- Defender STL gradient control ---
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
            elif role == 3:     # Chaser
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

            goal_dir_now = (goal_ii - pos_ii) / max(np.linalg.norm(goal_ii - pos_ii), EPS_PHYS)
            blocked_ahead = False
            for di in range(NUM_DEFENDERS):
                dvec_check = new_def[di] - pos_ii
                dd_check = np.linalg.norm(dvec_check)
                if dd_check < REPULSION_RANGE and np.dot(dvec_check, goal_dir_now) > 0:
                    blocked_ahead = True
                    break
            if blocked_ahead and intruder_force_cooldown[ii] <= 0:
                intruder_last_replan[ii] = step - WAYPOINT_REPLAN
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

            # --- Layer 4: Defender evade ---
            grad_evade = np.zeros(2)
            perp_goal = np.array([-grad_reach[1], grad_reach[0]])
            for di in range(NUM_DEFENDERS):
                dvec = pos_ii - new_def[di]
                dd = np.linalg.norm(dvec)
                if dd < REPULSION_RANGE and dd > EPS_PHYS:
                    overlap = (REPULSION_RANGE - dd) / REPULSION_RANGE
                    unit_away = dvec / dd
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
                    events.append(('say', f'Step {step + 1}: {INTRUDER_LABELS[ii]} REACHED goal!'))
                    events.append(('led', ii, LED_REACHED))
                elif (step + 1 > BLOCKED_CHECK_START_STEP
                        and np.linalg.norm(new_intr_all[ii] - curIntr[ii]) < BLOCKED_MOVE_THRESH):
                    intruder_active[ii] = False
                    intruder_blocked[ii] = True
                    events.append(('say', f'Step {step + 1}: {INTRUDER_LABELS[ii]} BLOCKED!'))
                    events.append(('led', ii, LED_NEUTRALIZED))
                    # A BLOCKED intruder is out of the game: land it now.
                    events.append(('land', ii))
                    intruder_landed[ii] = True

        cur_def = new_def
        cur_intr = new_intr_all

        frames.append(Frame(cur_def.copy(), cur_intr.copy(), ~intruder_landed, events))

        if not np.any(intruder_active):
            final_step = step + 2   # +1 for 0-index, +1 because this tick already advanced state
            break
    else:
        final_step = MAX_STEPS

    # ===================== Defenders return to their original positions =====================
    # (The intruders all land before this; stl_show does that, the table
    # only needs the defenders' legs.)
    if RETURN_TO_LAUNCH_PAD:
        home_targets = np.asarray(launch_pads, dtype=float)[:, :2].copy()
        home_desc = 'launch pads'
    else:
        home_targets = DEF_POS_INIT.copy()
        home_desc = 'start formation'

    returns = []
    all_home = False
    return_steps = 0
    for return_steps in range(1, RETURN_MAX_STEPS + 1):
        new_def = np.zeros((NUM_DEFENDERS, 2))
        all_home = True
        for di in range(NUM_DEFENDERS):
            to_home = home_targets[di] - cur_def[di]
            dist_home = np.linalg.norm(to_home)
            if dist_home < RETURN_ARRIVAL_DIST:
                new_def[di] = home_targets[di]
                continue
            all_home = False
            dir_home = to_home / dist_home

            push = np.zeros(2)
            for dj in range(NUM_DEFENDERS):
                if dj == di:
                    continue
                sep = cur_def[di] - cur_def[dj]
                sd = np.linalg.norm(sep)
                if EPS_PHYS < sd < RETURN_REPULSE_RANGE:
                    overlap = (RETURN_REPULSE_RANGE - sd) / RETURN_REPULSE_RANGE
                    push = push + overlap * (sep / sd)

            speed_scale = min(1.0, max(RETURN_MIN_SPEED_FRAC,
                                       dist_home / RETURN_BRAKE_DIST))
            travel = min(DEFENDER_SPEED * speed_scale, dist_home)
            step_vec = travel * dir_home + DEFENDER_SPEED * RETURN_REPULSE_GAIN * push

            for dj in range(NUM_DEFENDERS):
                if dj == di:
                    continue
                sep = cur_def[di] - cur_def[dj]
                sd = np.linalg.norm(sep)
                if EPS_PHYS < sd < RETURN_MIN_SEP:
                    toward = -sep / sd
                    closing = np.dot(step_vec, toward)
                    if closing > 0:
                        step_vec = step_vec - closing * toward

            step_norm = np.linalg.norm(step_vec)
            if step_norm > MAX_STEP_DIST:
                step_vec = step_vec * (MAX_STEP_DIST / step_norm)
            new_def[di] = clamp_to_arena(cur_def[di] + step_vec)

        cur_def = new_def
        returns.append(cur_def.copy())
        if all_home:
            break

    home_err = np.linalg.norm(cur_def - home_targets, axis=1)
    return_lines = [f'Return leg: {return_steps} steps ({return_steps * DT:.1f} s) | '
                    f'max home error = {np.max(home_err):.4f} m']
    if not all_home:
        return_lines.append(f'  WARNING: RETURN_MAX_STEPS ({RETURN_MAX_STEPS}) reached '
                            'before every defender was home -- landing from current '
                            'positions.')

    # ===================== Summary =====================
    summary = ['\n===== Complex Intruder STL 2D Summary =====',
               f'Total steps: {final_step} | Arena: {ARENA_W:.2f}x{ARENA_H:.2f} m',
               f'Speed boost: {SPEED_BOOST:.2f}x | tick: {DT:.3f} s',
               f'Intruder speed: {INTRUDER_SPEED_MIN:.5f}-{INTRUDER_SPEED_MAX:.5f} m/step '
               f'({INTRUDER_SPEED_MIN / DT:.3f}-{INTRUDER_SPEED_MAX / DT:.3f} m/s) '
               f'| Defender speed: {DEFENDER_SPEED:.5f} m/step '
               f'({DEFENDER_SPEED / DT:.3f} m/s)']
    n_blocked = int(np.sum(intruder_blocked))
    n_reached = int(np.sum(~intruder_active & ~intruder_blocked))
    n_still_active = int(np.sum(intruder_active))
    summary.append(f'Outcome: {n_blocked} BLOCKED | {n_reached} REACHED goal | '
                   f'{n_still_active} ACTIVE')
    for ii in range(NUM_INTRUDERS):
        final_dist = np.linalg.norm(cur_intr[ii] - INTRUDER_GOALS[ii])
        avg_spd = speed_sum[ii] / speed_count[ii] if speed_count[ii] > 0 else 0.0
        if intruder_blocked[ii]:
            status = 'BLOCKED'
        elif not intruder_active[ii]:
            status = 'REACHED'
        else:
            status = 'ACTIVE'
        summary.append(f'  {INTRUDER_LABELS[ii]} [{FLANK_LABELS[ii]}]: {status} | '
                       f'd_goal={final_dist:.4f} | dist={total_dist[ii]:.4f} | '
                       f'avg_spd={avg_spd:.5f}')

    return Game(seed=seed, def_roles=def_roles, header=header, frames=frames,
                returns=returns, home_desc=home_desc, return_lines=return_lines,
                summary=summary, final_step=final_step,
                intruder_blocked=intruder_blocked.copy(),
                intruder_active=intruder_active.copy())


# ===================== Verification =====================
# Not part of the original. Everything below reads the table; nothing in it
# changes what is flown.

#: Default separation floor for a plan, m. The game's own contact distances
#: are BLOCK_RADIUS / COLLISION_RADIUS = 0.20 m and 20 seeds of v3 measured
#: 0.18-0.19 m closest approach (2026-10-13), so the floor sits just under
#: that: it catches a regression, not the design. Rotor tip to rotor tip on a
#: Crazyflie 2.1 is ~0.13 m.
MIN_SEP_DEFAULT = 0.15


#: Peak / average speed of a rest-to-rest goTo. The firmware plans goTo as a
#: 7th-order polynomial with zero velocity, acceleration and jerk at both
#: ends (planner.c plan_go_to_from -> pptraj.c
#: piecewise_plan_7th_order_no_jerk), whose peak speed is 35/16 of the mean.
GOTO_PEAK_FACTOR = 35.0 / 16.0


@dataclass
class Check:
    """What :func:`verify` measured. ``problems`` empty means fly."""

    start_sep: float               # closest pair, takeoff columns + start move
    start_pair: tuple
    start_leg: float               # m, longest start leg
    start_leg_name: str
    start_peak: float              # m/s, that leg's peak speed
    game_sep: float                # closest pair, game + return
    game_pair: tuple
    game_when: str
    max_abs_xy: float              # m, furthest commanded |x| or |y|
    problems: list
    notes: list


def _pairs_min(names, pts):
    """Closest pair among named xy points; (distance, (a, b))."""
    best = (math.inf, ('', ''))
    for a in range(len(names)):
        for b in range(a + 1, len(names)):
            d = float(np.linalg.norm(pts[a] - pts[b]))
            if d < best[0]:
                best = (d, (names[a], names[b]))
    return best


def verify(game, def_names, intr_names, pads, min_sep=MIN_SEP_DEFAULT,
           max_speed=None):
    """Check the flight :func:`play` produced, from the drones' yaml marks.

    ``pads`` maps every name to its ``initial_position``. The start move is
    checked as what it is -- ten simultaneous rest-to-rest ``goTo`` legs, a
    straight line each, so the closed form is exact (the same algebra as
    ``safety.transition_min_sep``). The game and return legs are checked
    tick by tick in PLAN VIEW: every drone flies at one CRUISE_ALTITUDE, so
    plan view IS the 3D distance, and a landing intruder is kept in the set
    for LAND_DURATION after its land call (still under a defender's rotor
    wash on the way down). Between ticks a drone moves at most
    MAX_STEP_DIST = 0.025 m, which bounds what tick sampling can miss.

    ``max_speed`` (pass ``safety.MAX_SPEED``) turns a start leg whose PEAK
    speed exceeds it into a note: above it the controller lags the setpoint,
    and every separation number above assumes it does not.

    Deliberately NOT checked against ``safety``'s measured-arena radius: the
    game is built for its own 4 m x 4 m square and clamps to it.
    """
    names = list(def_names) + list(intr_names)
    src = np.array([np.asarray(pads[n], float)[:2] for n in names])
    dst = np.vstack([DEF_POS_INIT, INTRUDER_POS_INIT])
    problems, notes = [], []

    # takeoff columns (vertical, so the pads themselves) and the start move
    start_sep, start_pair = math.inf, ('', '')
    for a in range(len(names)):
        for b in range(a + 1, len(names)):
            d0 = src[a] - src[b]
            dv = (dst[a] - dst[b]) - d0
            den = float(np.dot(dv, dv))
            s = 0.0 if den < 1e-12 else min(1.0, max(0.0, -float(np.dot(d0, dv)) / den))
            d = float(np.linalg.norm(d0 + s * dv))
            if d < start_sep:
                start_sep, start_pair = d, (names[a], names[b])
    legs = np.linalg.norm(dst - src, axis=1)
    longest = int(np.argmax(legs))
    start_peak = GOTO_PEAK_FACTOR * float(legs[longest]) / INIT_MOVE_DURATION
    if max_speed is not None and start_peak > max_speed:
        notes.append(
            f'start move: {names[longest]} flies {legs[longest]:.2f} m in '
            f'{INIT_MOVE_DURATION:g} s, peak {start_peak:.2f} m/s, over the '
            f'{max_speed:g} m/s the controller tracks (safety.MAX_SPEED). It '
            'will lag its setpoint; park it nearer its start slot')
    if start_sep < min_sep:
        problems.append(
            f'start move: {start_pair[0]} and {start_pair[1]} pass {start_sep:.3f} m '
            f'apart (floor {min_sep:.2f} m) flying from their yaml marks to the '
            'start formation')

    # the game, tick by tick
    game_sep, game_pair, game_when = math.inf, ('', ''), ''
    land_tick = {}
    linger = int(round(LAND_DURATION / DT))
    max_abs = 0.0
    for k, fr in enumerate(game.frames):
        for ev in fr.events:
            if ev[0] == 'land':
                land_tick[ev[1]] = k
        idx = [ii for ii in range(NUM_INTRUDERS)
               if ii not in land_tick or k - land_tick[ii] <= linger]
        nm = list(def_names) + [intr_names[ii] for ii in idx]
        pts = np.vstack([fr.defenders, fr.intruders[idx]])
        d, pair = _pairs_min(nm, pts)
        if d < game_sep:
            game_sep, game_pair = d, pair
            game_when = f'game tick {k + 1} (t+{(k + 1) * DT:.2f} s)'
            # A landing intruder is still at cruise height for the first
            # second or so (the descent is a 7th-order polynomial: ~1 mm in
            # the first 0.35 s). The defenders' keep-out from it is
            # INTR_KEEP_OUT, so this is where the closest approaches are.
            for ii in idx:
                if ii in land_tick and intr_names[ii] in pair:
                    game_when += (f', {intr_names[ii]} landing for '
                                  f'{(k - land_tick[ii]) * DT:.2f} s')
        max_abs = max(max_abs, float(np.max(np.abs(pts))))
    # the return leg: the intruders are all down by then
    for k, xy in enumerate(game.returns):
        d, pair = _pairs_min(list(def_names), xy)
        if d < game_sep:
            game_sep, game_pair, game_when = d, pair, f'return tick {k + 1}'
        max_abs = max(max_abs, float(np.max(np.abs(xy))))
    if game_sep < min_sep:
        problems.append(f'{game_when}: {game_pair[0]} and {game_pair[1]} commanded '
                        f'{game_sep:.3f} m apart (floor {min_sep:.2f} m)')

    return Check(start_sep, start_pair, float(legs[longest]), names[longest], start_peak,
                 game_sep, game_pair, game_when, max_abs, problems, notes)


def fly(game, def_cfs, intr_cfs, timeHelper, say=print):
    """Send the table to the drones, in the original's order and timing.

    Arms, takes off, flies the start move, streams every tick, lands the
    intruders, walks the defenders home, lands them and disarms -- the
    original's ``main()`` from the arm loop on, call for call. The caller
    owns the abort: it must set its ``armed`` flag BEFORE calling this
    (WRITING-A-SHOW.md 1.1 rule 2), because the first thing this does is arm.

    Ticks are paced to ``t0 + k * DT`` rather than "DT after the last
    sleep", so per-tick command latency cannot accumulate over a game; with
    no latency (a stub clock) the two are the same.
    """
    mission_cfs = list(def_cfs) + list(intr_cfs)

    def led(cf, hex_color):
        if not ENABLE_LEDS:
            return
        try:                       # cosmetic: must never stop the show
            cf.setParam('colorLedBot.wrgb8888', led_value(hex_color))
        except Exception as exc:   # noqa: BLE001
            say(f'  (LED set failed for {cf.prefix}: {exc})')

    for line in game.header:
        say(line)

    for cf in mission_cfs:
        cf.arm(True)
    timeHelper.sleep(1.0)

    for cf in mission_cfs:
        cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    for di, cf in enumerate(def_cfs):
        cf.goTo([DEF_POS_INIT[di, 0], DEF_POS_INIT[di, 1], CRUISE_ALTITUDE],
                GOTO_YAW, INIT_MOVE_DURATION)
    for ii, cf in enumerate(intr_cfs):
        cf.goTo([INTRUDER_POS_INIT[ii, 0], INTRUDER_POS_INIT[ii, 1], CRUISE_ALTITUDE],
                GOTO_YAW, INIT_MOVE_DURATION)
    timeHelper.sleep(INIT_MOVE_DURATION)

    for cf in def_cfs:
        led(cf, LED_DEFENDER)
    for cf in intr_cfs:
        led(cf, LED_INTRUDER)

    say(f'GAME START - seed {game.seed}, {len(game.frames)} ticks '
        f'({game.game_s:.1f} s) planned')
    landed = np.zeros(NUM_INTRUDERS, dtype=bool)
    t0 = timeHelper.time()
    for k, fr in enumerate(game.frames):
        for ev in fr.events:
            if ev[0] == 'say':
                say(ev[1])
            elif ev[0] == 'led':
                led(intr_cfs[ev[1]], ev[2])
            elif ev[0] == 'land':
                intr_cfs[ev[1]].land(targetHeight=LAND_TARGET_HEIGHT, duration=LAND_DURATION)
                landed[ev[1]] = True
        for di, cf in enumerate(def_cfs):
            cf.goTo([fr.defenders[di, 0], fr.defenders[di, 1], CRUISE_ALTITUDE],
                    GOTO_YAW, DT)
        for ii, cf in enumerate(intr_cfs):
            if not fr.streaming[ii]:
                continue
            cf.goTo([fr.intruders[ii, 0], fr.intruders[ii, 1], CRUISE_ALTITUDE],
                    GOTO_YAW, DT)
        timeHelper.sleep(max(0.0, t0 + (k + 1) * DT - timeHelper.time()))

    for ii, cf in enumerate(intr_cfs):
        if not landed[ii]:
            cf.land(targetHeight=LAND_TARGET_HEIGHT, duration=LAND_DURATION)
            landed[ii] = True
    timeHelper.sleep(LAND_DURATION)

    say(f'\nAll intruders resolved -- defenders returning to {game.home_desc}...')
    t0 = timeHelper.time()
    for k, xy in enumerate(game.returns):
        for di, cf in enumerate(def_cfs):
            cf.goTo([xy[di, 0], xy[di, 1], CRUISE_ALTITUDE], GOTO_YAW, DT)
        timeHelper.sleep(max(0.0, t0 + (k + 1) * DT - timeHelper.time()))
    for line in game.return_lines:
        say(line)
    timeHelper.sleep(RETURN_SETTLE_TIME)

    for cf in def_cfs:
        cf.land(targetHeight=LAND_TARGET_HEIGHT, duration=LAND_DURATION)
    timeHelper.sleep(LAND_DURATION)
    for cf in mission_cfs:
        cf.arm(False)

    for line in game.summary:
        say(line)


def fingerprint(path):
    """sha256 of a file, to tell whether it is the version this port came from."""
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()
