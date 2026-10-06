#!/usr/bin/env python3
"""Ask the rig whether it will fly, before anything is uploaded or armed.

Lived in ``constellation_show`` until 2026-10-06, which meant the constellation
was the only show that ran it: the carousel, the starter show and the escort
demo all went straight from a geometry check to ``cf.arm(True)``.
"""

import time

import rclpy
from crazyflie_interfaces.msg import Status
from rclpy.qos import qos_profile_sensor_data

#: Below this the drone is refused outright.
BATTERY_REFUSE_V = 3.7
#: Below this it flies, but the operator is told.
BATTERY_WARN_V = 3.8


def check_supervisor(node, names, timeout=6.0):
    """Refuse to fly drones the firmware will not arm. Runs BEFORE the upload.

    MEASURED 2026-10-02: five drones sat in the supervisor's LOCKED state, which
    an emergency stop latches and which **only a power cycle clears** -- the
    firmware's state machine has exactly one transition out of locked, back to
    locked, blocked always. Nothing in the flight path noticed: the show checked
    placement, spent 55 s uploading trajectories, and only then would it have
    failed at arm(). The operator read the silent upload as a hang and killed
    it, which looked like a bug in the show.

    So: ask every drone what the supervisor thinks, before anything is
    uploaded. Battery thresholds match the console's health page.

    Returns ``(problems, notes)``. A non-empty ``problems`` means do not fly.
    A drone whose ``/status`` never arrives is a NOTE, not a problem, because
    old firmware does not publish it -- but see the caveat below.

    CAVEAT, and why callers should also check altitude after takeoff: a silent
    ``/status`` is exactly what the telemetry stall produces (CLAUDE.md, "Link
    alive, log data dead"), so a stalled drone passes this check by saying
    nothing. ``arm()`` is fire-and-forget on the client side -- the service
    response is never inspected -- so a refused arm is invisible. The only
    positive confirmation that a drone actually flew is its mocap altitude.
    """
    latest = {}

    def mk(nm):
        def cb(msg):
            latest[nm] = msg
        return cb

    subs = [node.create_subscription(Status, f'/{nm}/status', mk(nm),
                                     qos_profile_sensor_data) for nm in names]
    deadline = time.time() + timeout            # /status is published at 1 Hz
    while time.time() < deadline and len(latest) < len(names):
        rclpy.spin_once(node, timeout_sec=0.1)
    for sub in subs:
        node.destroy_subscription(sub)

    problems, notes = [], []
    for nm in names:
        st = latest.get(nm)
        if st is None:
            notes.append(f'{nm}: no /{nm}/status in {timeout:g} s - cannot check '
                         'the supervisor (old firmware, a dead link, or the '
                         'telemetry stall). Watch this one on takeoff')
            continue
        info, v = st.supervisor_info, st.battery_voltage
        if info & Status.SUPERVISOR_INFO_IS_LOCKED:
            problems.append(f'{nm}: supervisor LOCKED (0x{info:04x}) - where an '
                            'E-STOP leaves a drone. Battery out and in; nothing '
                            'else clears it')
        elif not info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
            problems.append(f'{nm}: the firmware says it cannot be armed '
                            f'(0x{info:04x}) - preflight checks have not passed')
        if info & Status.SUPERVISOR_INFO_IS_TUMBLED:
            problems.append(f'{nm}: tumbled - stand it back on its feet')
        # 0.00 V is not a healthy battery, it is a log block that has not
        # delivered a value yet -- do not report it as "ready".
        if not v:
            notes.append(f'{nm}: no battery reading yet (0.00 V) - the voltage '
                         'log has not arrived; check the preflight GUI')
        elif v < BATTERY_REFUSE_V:
            problems.append(f'{nm}: battery {v:.2f} V, below {BATTERY_REFUSE_V} V - swap it')
        elif v < BATTERY_WARN_V:
            notes.append(f'{nm}: battery {v:.2f} V - low for a full show')
        else:
            notes.append(f'{nm}: ready, battery {v:.2f} V, rssi {st.rssi}')
    return problems, notes


def report_supervisor(node, names, timeout=6.0):
    """:func:`check_supervisor`, printed, returning True if it is safe to fly.

    The shared "SUPERVISOR CHECK" block every show printed by hand.
    """
    print('  SUPERVISOR CHECK (what the firmware will let us do)')
    problems, notes = check_supervisor(node, names, timeout)
    for line in notes:
        print(f'    {line}')
    if problems:
        print('\n  REFUSING TO FLY - the drones cannot be armed:', flush=True)
        for p in problems:
            print(f'    - {p}')
        print('\n  Nothing was uploaded. Fix the above and re-run; '
              '`dry_run:=true` re-checks without uploading.\n')
        return False
    print('    all drones armable\n')
    return True
