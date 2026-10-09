#!/usr/bin/env python3
"""Stop a show without leaving the drones flying their last command.

Every entry point in this package that can call ``cf.arm(True)`` must use this
module. It was written for ``constellation_show`` and lived there until
2026-10-06, which meant the carousel (``swarm_show``) and the starter show
(``demo_show``) -- the two that have actually flown on hardware -- had **no
abort path at all**: a Ctrl-C mid-figure left five armed drones airborne with
no script and no landing, and the operator's only remaining answer was the
broadcast E-STOP, which cuts motors and drops them.

The E-STOP is still the right answer to a drone that is out of control. This
is the answer to a *script* that stopped.

Why this is not just ``try: ... except KeyboardInterrupt: allcfs.land()``
-----------------------------------------------------------------------
``rclpy.init`` (called inside ``Crazyswarm()``) installs its own SIGINT
handler that shuts the ROS context down *before* the exception reaches the
script. The landing call then dies with "failed to initialize wait set: the
given context is not valid" -- at exactly the moment it is needed. Verified
failing in sim 2026-09-20. :func:`take_signals` takes the disposition back
after ``Crazyswarm()`` has returned, which keeps the context alive long enough
to command a landing.

Testing an abort
----------------
``ros2 run`` does NOT forward a signal sent to it alone, so send the signal to
the script's own process::

    kill -INT $(pgrep -f swarm_show)

A terminal Ctrl-C reaches the child because it goes to the whole foreground
process group, which is why Ctrl-C works interactively and ``kill -INT`` on
the wrapper appears to do nothing.
"""

import signal

ABORT_LAND_DURATION = 4.0      # s -- a gentle descent from wherever they are


class ShowAborted(BaseException):
    """Raised by the signal handler installed in :func:`take_signals`.

    A BaseException, like KeyboardInterrupt and for the same reason: the
    signal arrives wherever the script happens to be, and any ``except
    Exception`` in its path -- a guarded select(), a marker publish, a light
    cue -- would otherwise swallow the abort and keep flying. It was an
    Exception until 2026-10-09, when exactly that was measured in sim: a
    SIGINT at escort_show's paced ARM gate was eaten by the gate's own
    ``except Exception`` and the show sat waiting. Every show catches it by
    name before its ``except BaseException``, so nothing else changes.
    """


def take_signals():
    """Handle SIGINT/SIGTERM ourselves so an abort can still command a landing.

    MEASURED, sim, 2026-09-20: without this, Ctrl-C mid-show left the drones
    flying. See the module docstring for why rclpy's own handler is the
    problem.

    SIGTERM is covered as well as SIGINT because the mission console's
    Processes -> Stop escalates SIGINT -> SIGTERM -> SIGKILL
    (``console/mission_console/procs.py``; a flight script gets 15 s before the
    SIGTERM) -- an abort landing has to start on the first signal to have any
    chance of finishing. The mission window's Abort sends one SIGINT only.

    A second Ctrl-C during the abort restores the default handler and kills the
    process outright: the operator must always be able to give up on the
    script and reach for the E-STOP.
    """
    def handler(signum, _frame):
        raise ShowAborted('Ctrl-C' if signum == signal.SIGINT else f'signal {signum}')
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def abort_land(allcfs, cfs, timeHelper, land_height, why):
    """Land where they are and disarm. Best effort: report what worked.

    ``land_height`` is a float (metres), not a show config -- every caller used
    to pass a whole config object for this one field, and ``escort_show`` had
    to invent a stub class to satisfy it.

    Restores the DEFAULT disposition for both signals we claimed, not just
    SIGINT, so a second Ctrl-C (or SIGTERM) kills the process outright -- the
    operator can always give up on the script and reach for the E-STOP. Note
    what that means for a SIGTERM that arrives DURING the landing: it no longer
    raises into the disarm loop (what used to happen), it terminates the
    process, which can equally leave drones down but armed. The console
    therefore waits 15 s after SIGINT before sending SIGTERM to a flight
    script, and the mission window's Abort never sends one.
    """
    print(f'\n  *** ABORT ({why}) - landing all drones where they are ***', flush=True)
    for sig in (signal.SIGINT, signal.SIGTERM):     # a second signal kills us
        signal.signal(sig, signal.SIG_DFL)
    try:
        allcfs.land(targetHeight=land_height, duration=ABORT_LAND_DURATION)
        timeHelper.sleep(ABORT_LAND_DURATION + 0.5)
        for cf in cfs:
            cf.arm(False)
        print('  landed and disarmed.\n', flush=True)
    except BaseException as e:                     # noqa: BLE001
        print(f'  could NOT complete the abort landing ({e!r}).\n'
              '  Use the E-STOP (console or preflight GUI "e") if a drone is '
              'still airborne.\n', flush=True)
