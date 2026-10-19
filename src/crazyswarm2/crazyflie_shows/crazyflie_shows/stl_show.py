#!/usr/bin/env python3
"""STL pursuit-evasion: 5 defenders vs 5 intruders, the colleague's game, flown.

    ros2 run crazyflie_shows stl_show --ros-args -p use_sim_time:=true   # sim
    ros2 run crazyflie_shows stl_show                                    # hardware
    ros2 run crazyflie_shows stl_show --ros-args -p dry_run:=true        # rehearsal
    ros2 run crazyflie_shows stl_show --ros-args -p seed:=1234           # replay a run

The algorithm is ``stl_game`` (a copy of the colleague's v3 script, which
stays unchanged in crazyflie_examples). This file adds what every show in this
package must do before it arms (docs/WRITING-A-SHOW.md section 1): play the
game for the chosen seed and check the table that will fly, prove mocap
tracks every drone and that each sits on its ``initial_position``, ask the
firmware supervisor, and land on Ctrl-C / the console's Stop instead of
leaving ten drones on their last setpoint. Design: crazyflie_shows/STL.md (package root).

Every run prints its seed. The game is open loop, so the same seed is the
same flight: ``plan_stl --seed N`` checks it offline, ``seed:=N`` flies it.

Parameters (``--ros-args -p name:=value``)
------------------------------------------
=====================  ==========  ==========================================
``dry_run``            false       every check, then exit. Never arms.
``seed``               -1          the game's random seed; -1 draws one
``min_sep``            0.15        m, refuse a plan closer than this
``check_placement``    true        mocap vs initial_position, before arming
``placement_tol``      0.25        m, how far off a drone may be
``defenders``          (the game)  comma list of 5 names, slot order
``intruders``          (the game)  comma list of 5 names, slot order
=====================  ==========  ==========================================
"""

import secrets
import sys

import numpy as np
from crazyflie_py import Crazyswarm

from crazyflie_shows import safety, stl_game
from crazyflie_shows.abort import ShowAborted, abort_land, take_signals
from crazyflie_shows.preflight import report_supervisor
from crazyflie_shows.swarm_show import _param, check_placement, wait_for_mocap


class _Starts:                  # check_placement only reads .starts
    def __init__(self, starts):
        self.starts = starts


def _names(node, name, default):
    raw = str(_param(node, name, ','.join(default)))
    return [s.strip() for s in raw.split(',') if s.strip()]


def main():
    swarm = Crazyswarm()
    timeHelper, allcfs = swarm.timeHelper, swarm.allcfs
    node = allcfs
    cfs = allcfs.crazyflies

    # 1. parameters --------------------------------------------------------
    sim = bool(_param(node, 'use_sim_time', False))
    dry_run = bool(_param(node, 'dry_run', False))
    seed = int(_param(node, 'seed', -1))
    min_sep = float(_param(node, 'min_sep', stl_game.MIN_SEP_DEFAULT))
    want_placement = bool(_param(node, 'check_placement', True))
    tol = float(_param(node, 'placement_tol', 0.25))
    def_names = _names(node, 'defenders', stl_game.DEFENDER_CF_NAMES)
    intr_names = _names(node, 'intruders', stl_game.INTRUDER_CF_NAMES)
    if seed < 0:
        seed = secrets.randbelow(2 ** 31)

    # 2. the fleet the game needs, from the live stack ----------------------
    if (len(def_names), len(intr_names)) != (stl_game.NUM_DEFENDERS, stl_game.NUM_INTRUDERS):
        raise SystemExit(f'the game is {stl_game.NUM_DEFENDERS} v {stl_game.NUM_INTRUDERS}; '
                         f'got {len(def_names)} defenders and {len(intr_names)} intruders')
    by_name = allcfs.crazyfliesByName
    names = def_names + intr_names
    missing = [n for n in names if n not in by_name]
    if missing:
        raise SystemExit(
            f'not on the server: {", ".join(missing)}. Enable them in '
            'crazyflie/config/crazyflies.yaml (`enabled: true`) and restart the launch '
            '(the server reads the yaml only at start).')
    def_cfs = [by_name[n] for n in def_names]
    intr_cfs = [by_name[n] for n in intr_names]
    idle = sorted(set(by_name) - set(names))
    if idle:
        print(f'  NOTE enabled but not in the game (stay on the ground): {", ".join(idle)}')

    # 3. play the game for this seed and check the table that will fly ------
    pads = {n: np.asarray(by_name[n].initialPosition, float) for n in names}
    launch = np.array([pads[n][:2] for n in def_names])
    print(f'\n  STL GAME  seed {seed}  (replay: -p seed:={seed}; offline: plan_stl --seed {seed})')
    print(f'  defenders {",".join(def_names)}   intruders {",".join(intr_names)}')
    game = stl_game.play(seed, launch_pads=launch)
    chk = stl_game.verify(game, def_names, intr_names, pads, min_sep,
                          max_speed=safety.MAX_SPEED)
    print(f'  planned: {game.outcome}  game {game.game_s:.1f} s, return {game.return_s:.1f} s')
    print(f'  start move: closest {chk.start_sep:.3f} m '
          f'({chk.start_pair[0]}-{chk.start_pair[1]}), peak {chk.start_peak:.2f} m/s '
          f'({chk.start_leg_name})')
    print(f'  game: closest {chk.game_sep:.3f} m ({chk.game_pair[0]}-{chk.game_pair[1]}, '
          f'{chk.game_when})')
    for line in chk.notes:
        print(f'  NOTE {line}')
    if chk.problems:
        print(f'\n  REFUSING TO FLY - the plan breaks the {min_sep:.2f} m floor:')
        for p in chk.problems:
            print(f'    - {p}')
        print('\n  Nothing armed. plan_stl shows every seed; the floor is the '
              'min_sep parameter.\n')
        return 1
    print(f'  plan ok against the {min_sep:.2f} m floor\n')

    # 4. is each drone tracked, and where the start move assumes? -----------
    if want_placement and not sim:
        print('  MOCAP CHECK (every flying drone must be on /poses)')
        live = wait_for_mocap(node, names)
        print('    all tracked\n')
        print('  PLACEMENT CHECK (mocap vs initial_position)')
        check_placement(node, def_cfs + intr_cfs, names,
                        _Starts([pads[n] for n in names]), tol, live=live)
        print('    all drones within tolerance\n')
    elif sim:
        print('  placement check skipped: backend:=sim publishes no /poses\n')
    else:
        print('  *** placement check DISABLED by parameter ***\n')

    # 5. supervisor go/no-go -- read-only, so before the dry_run exit -------
    if not sim and not report_supervisor(node, names):
        return 1

    if dry_run:
        print('  dry_run - plan and preflight checks done; nothing armed.\n')
        return 0

    # 6. fly, under the abort ----------------------------------------------
    armed = False
    try:
        take_signals()
        armed = True                      # BEFORE the arm loop inside fly()
        stl_game.fly(game, def_cfs, intr_cfs, timeHelper,
                     say=lambda s: print(s, flush=True))
        armed = False
        print('\n  landed, disarmed - done\n')
        return 0
    except ShowAborted as e:
        if armed:
            abort_land(allcfs, cfs, timeHelper, stl_game.LAND_TARGET_HEIGHT, str(e))
        return 130
    except BaseException as e:            # noqa: BLE001 -- real fault: land, then re-raise
        if armed:
            abort_land(allcfs, cfs, timeHelper, stl_game.LAND_TARGET_HEIGHT, repr(e))
        raise


if __name__ == '__main__':
    sys.exit(main())
