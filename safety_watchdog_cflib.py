#!/usr/bin/env python3
"""Standalone MOCAP-FREE safety watchdog for a Crazyflie — no ROS, no mocap.

Why not mocap: with estimator=2 (Kalman) the onboard position estimate
stateEstimate.x/y/z is FUSED with the mocap pose the server sends. If the mocap
frame drifts relative to the drone, that estimate drifts too — so a geofence
built on it is worthless exactly when you need it. This watchdog therefore never
looks at position. It watches only signals that come straight off the IMU /
barometer / firmware supervisor and are independent of any external tracking:

  * tilt        stabilizer.roll / stabilizer.pitch   (attitude est: gyro+accel)
  * angular rate gyro.x / gyro.y / gyro.z            (raw rate gyro)
  * tumble/crash supervisor.info bits                (firmware's own verdict)
  * thrust      stabilizer.thrust                    (controller flooring throttle)
  * altitude    baro.asl                             (barometric, vs. takeoff ref)
  * link/battery packet timing / pm.vbat

A runaway from mocap drift shows up here as violent behaviour — the controller
chases a bad setpoint, so tilt / rate / thrust spike and the supervisor flags a
tumble. We catch the SYMPTOM, which is mocap-free, not the position (which isn't).

Escalation while airborne:
  * SOFT limit  (tilt/rate/thrust/baro-ceiling/battery) -> controlled land
                 straight down (no go_to: without position we cannot hold xy),
                 then disarm.
  * HARD limit  (tumble, crash, extreme tilt/rate, hard ceiling, telemetry gone)
                 -> immediate motor kill (supervisor emergency stop).

Ctrl+C once while airborne = controlled land; Ctrl+C twice = motor kill.

NOTE: horizontal position is deliberately NOT fenced — without mocap or a flow/
positioning deck nothing on the drone knows its true xy, so no script can. This
guards against violent runaways, not slow sideways drift into a wall; for that
you need a flow deck / UWB and a different design.

IMPORTANT: one process per Crazyradio. Do NOT run this while the crazyswarm2
server or cfclient is connected to the same drone. Use the system python3.12
(cflib lives in ~/.local/lib/python3.12, not conda):

    /usr/bin/python3 safety_watchdog_cflib.py                    # defaults, cf3
    /usr/bin/python3 safety_watchdog_cflib.py --uri radio://0/80/2M/E7E7E7E701
    /usr/bin/python3 safety_watchdog_cflib.py --dry-run --assume-flying   # bench
"""
import argparse
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

MONITOR, LANDING, DONE = 'MONITOR', 'LANDING', 'DONE'

# supervisor.info bit positions (cflib/crazyflie/supervisor.py)
BIT_IS_FLYING = 4
BIT_IS_TUMBLED = 5
BIT_IS_CRASHED = 7


class SafetyWatchdog:
    def __init__(self, cf, args):
        self.cf = cf
        self.args = args
        self.att = None            # (roll, pitch) deg
        self.rate = None           # (gx, gy, gz) deg/s
        self.thrust = 0.0          # 0..65535
        self.asl = None            # barometric altitude, m
        self.asl_ref = None        # takeoff reference altitude, m
        self.vbat = None
        self.info = 0              # supervisor bitfield
        self.last_imu = None       # wall time of last attitude/rate packet
        self.last_status = 0.0
        self.low_batt_since = None
        self.thrust_sat_since = None
        self.flying = False
        self.state = MONITOR
        self.state_since = None
        self.land_deadline = None
        self.link_ok = True
        cf.connection_lost.add_callback(self._on_link_lost)
        cf.disconnected.add_callback(self._on_link_lost)

    # ---------------------------------------------------------------- inputs

    def start_logging(self):
        # two blocks, each <= 26-byte payload limit (LogConfig.MAX_LEN)
        lg_att = LogConfig(name='safety_att', period_in_ms=50)   # 5 floats = 20 B
        for v in ('stabilizer.roll', 'stabilizer.pitch',
                  'gyro.x', 'gyro.y', 'gyro.z'):
            lg_att.add_variable(v, 'float')
        lg_att.data_received_cb.add_callback(self._on_att)

        lg_sys = LogConfig(name='safety_sys', period_in_ms=50)   # 3f + u16 = 14 B
        lg_sys.add_variable('stabilizer.thrust', 'float')
        lg_sys.add_variable('baro.asl', 'float')
        lg_sys.add_variable('pm.vbat', 'float')
        lg_sys.add_variable('supervisor.info', 'uint16_t')
        lg_sys.data_received_cb.add_callback(self._on_sys)

        for lg in (lg_att, lg_sys):
            self.cf.log.add_config(lg)
            lg.start()

    def _on_att(self, _ts, data, _lg):
        self.att = (data['stabilizer.roll'], data['stabilizer.pitch'])
        self.rate = (data['gyro.x'], data['gyro.y'], data['gyro.z'])
        self.last_imu = time.monotonic()

    def _on_sys(self, _ts, data, _lg):
        self.thrust = data['stabilizer.thrust']
        self.asl = data['baro.asl']
        self.vbat = data['pm.vbat']
        self.info = int(data['supervisor.info'])
        if self.asl_ref is None:
            self.asl_ref = self.asl   # first reading = ground reference

    def _on_link_lost(self, uri, msg=''):
        if self.link_ok and self.state != DONE:   # DONE = we closed it ourselves
            self.link_ok = False
            print(f'RADIO LINK LOST ({uri}): {msg} — firmware failsafe is in charge now')

    # ------------------------------------------------------------- accessors

    def _bit(self, pos):
        return bool((self.info >> pos) & 1)

    @property
    def rel_alt(self):
        if self.asl is None or self.asl_ref is None:
            return 0.0
        return self.asl - self.asl_ref

    def update_flying(self):
        was = self.flying
        firmware_flying = self._bit(BIT_IS_FLYING)
        baro_flying = self.rel_alt > self.args.flight_min
        self.flying = self.args.assume_flying or firmware_flying or baro_flying
        if self.flying != was:
            print('Airborne — limits active' if self.flying else 'On ground')
        if not self.flying:
            # reset ground-referenced trackers so the next flight starts clean
            self.asl_ref = self.asl
            self.low_batt_since = None
            self.thrust_sat_since = None

    # ------------------------------------------------------------ monitoring

    def _maybe_status(self, now):
        if not (self.args.status or self.args.dry_run):
            return
        if now - self.last_status < 1.0 or self.att is None:
            return
        self.last_status = now
        roll, pitch = self.att
        rate = max(abs(r) for r in self.rate) if self.rate else 0.0
        vbat = f'{self.vbat:.2f}V' if self.vbat is not None else '  ?  '
        print(f'  [status] tilt=({roll:+.0f},{pitch:+.0f})° rate={rate:.0f}°/s '
              f'thr={self.thrust / 65535 * 100:.0f}% alt={self.rel_alt:+.2f}m '
              f'{vbat} flying={self.flying} tumbled={self._bit(BIT_IS_TUMBLED)}')

    def check(self):
        """One 20 Hz tick of the state machine. Returns False when finished."""
        if self.state == DONE or not self.link_ok:
            return False
        now = time.monotonic()
        self.update_flying()
        self._maybe_status(now)

        if self.flying and self.last_imu is not None:
            gap = now - self.last_imu
            if gap > self.args.timeout:
                self.emergency(f'telemetry lost for {gap * 1000:.0f} ms mid-flight')
                return True

        if not self.flying or self.att is None:
            return True

        hard = self.hard_violation()
        if hard:
            self.emergency(hard)
            return True

        if self.state == MONITOR:
            soft = self.soft_violation(now)
            if soft:
                print(f'LIMIT EXCEEDED: {soft} -> landing')
                self.start_landing(now)
        elif self.state == LANDING:
            self.continue_landing(now)
        return True

    def hard_violation(self):
        a = self.args
        if self._bit(BIT_IS_TUMBLED):
            return 'supervisor reports TUMBLED'
        if self._bit(BIT_IS_CRASHED):
            return 'supervisor reports CRASHED'
        roll, pitch = self.att
        if abs(roll) > a.tilt_kill or abs(pitch) > a.tilt_kill:
            return f'tilt {max(abs(roll), abs(pitch)):.0f}° > {a.tilt_kill}° (tumble)'
        rate = max(abs(r) for r in self.rate)
        if rate > a.rate_kill:
            return f'angular rate {rate:.0f}°/s > {a.rate_kill}°/s (spin)'
        if self.rel_alt > a.z_kill:
            return f'altitude {self.rel_alt:.2f} m above hard ceiling {a.z_kill} m'
        return None

    def soft_violation(self, now):
        a = self.args
        roll, pitch = self.att
        if abs(roll) > a.tilt_max or abs(pitch) > a.tilt_max:
            return f'tilt {max(abs(roll), abs(pitch)):.0f}° (limit {a.tilt_max}°)'
        rate = max(abs(r) for r in self.rate)
        if rate > a.rate_max:
            return f'angular rate {rate:.0f}°/s (limit {a.rate_max}°/s)'
        if self.rel_alt > a.z_max:
            return f'altitude {self.rel_alt:.2f} m (ceiling {a.z_max} m)'
        # sustained max thrust = controller fighting a bad estimate
        if self.thrust / 65535.0 > a.thrust_sat:
            if self.thrust_sat_since is None:
                self.thrust_sat_since = now
            elif now - self.thrust_sat_since > a.thrust_sat_time:
                return (f'thrust saturated {self.thrust / 65535 * 100:.0f}% for '
                        f'{a.thrust_sat_time:.0f} s')
        else:
            self.thrust_sat_since = None
        # sustained low battery (brief sag under load is normal)
        if self.vbat is not None and self.vbat < a.vbat_land:
            if self.low_batt_since is None:
                self.low_batt_since = now
            elif now - self.low_batt_since > 2.0:
                return f'battery {self.vbat:.2f} V < {a.vbat_land} V for 2 s'
        else:
            self.low_batt_since = None
        return None

    # ------------------------------------------------------------- responses

    def start_landing(self, now):
        duration = max(self.args.land_min_duration,
                       max(self.rel_alt, 0.0) / self.args.land_speed)
        self.state, self.state_since = LANDING, now
        self.land_deadline = now + duration + 2.0
        print(f'LANDING from {self.rel_alt:.2f} m, {duration:.1f} s descent')
        if self.args.dry_run:
            print('[dry-run] would notify_setpoint_stop + land')
            return
        self.cf.commander.send_notify_setpoint_stop()
        self.cf.high_level_commander.land(self.args.land_height, duration)

    def continue_landing(self, now):
        on_ground = (not self._bit(BIT_IS_FLYING)
                     and self.rel_alt < self.args.land_height + 0.10)
        if not on_ground and now < self.land_deadline:
            return
        if not on_ground:
            # descent stalled — cut motors low rather than let it carry away
            self.emergency(f'still airborne ({self.rel_alt:.2f} m) after land deadline')
            return
        self.state = DONE
        print('Landed. Stopping motors and disarming.')
        if not self.args.dry_run:
            self.cf.high_level_commander.stop()
            time.sleep(0.1)
            self.cf.supervisor.send_arming_request(False)

    def emergency(self, reason):
        self.state = DONE
        print(f'EMERGENCY STOP: {reason} -> cutting motors')
        if self.args.dry_run:
            print('[dry-run] would send emergency stop')
            return
        for _ in range(3):   # CRTP is reliable; repeats are belt-and-braces
            self.cf.supervisor.send_emergency_stop()
            time.sleep(0.02)

    def request_land(self):
        """Manual Ctrl+C: controlled land from wherever we are."""
        if self.state in (LANDING, DONE) or not self.flying:
            self.state = DONE
            return
        print('Manual land requested')
        self.start_landing(time.monotonic())


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--uri', default='radio://0/80/2M/E7E7E7E703',
                   help='crazyflie URI (default cf3)')
    p.add_argument('--tilt-max', type=float, default=45.0,
                   help='soft roll/pitch limit -> land, degrees (default 45)')
    p.add_argument('--tilt-kill', type=float, default=65.0,
                   help='hard tilt -> motor kill, degrees (default 65)')
    p.add_argument('--rate-max', type=float, default=250.0,
                   help='soft angular-rate limit -> land, deg/s (default 250)')
    p.add_argument('--rate-kill', type=float, default=500.0,
                   help='hard angular rate -> motor kill, deg/s (default 500)')
    p.add_argument('--z-max', type=float, default=1.2,
                   help='soft baro ceiling above takeoff -> land, m (default 1.2)')
    p.add_argument('--z-kill', type=float, default=1.8,
                   help='hard baro ceiling -> motor kill, m (default 1.8)')
    p.add_argument('--thrust-sat', type=float, default=0.90,
                   help='thrust fraction counted as saturated (default 0.90)')
    p.add_argument('--thrust-sat-time', type=float, default=1.5,
                   help='seconds of saturated thrust before landing (default 1.5)')
    p.add_argument('--vbat-land', type=float, default=3.3,
                   help='land if battery below this for 2 s, volts (default 3.3)')
    p.add_argument('--timeout', type=float, default=0.5,
                   help='max telemetry gap mid-flight before motor kill, s (default 0.5)')
    p.add_argument('--flight-min', type=float, default=0.1,
                   help='baro altitude above takeoff counted as airborne, m (default 0.1)')
    p.add_argument('--land-height', type=float, default=0.04,
                   help='land target height, m (default 0.04)')
    p.add_argument('--land-speed', type=float, default=0.3,
                   help='descent speed used to size the land duration, m/s (default 0.3)')
    p.add_argument('--land-min-duration', type=float, default=2.0,
                   help='minimum land duration, s (default 2)')
    p.add_argument('--assume-flying', action='store_true',
                   help='force airborne state (BENCH TESTING ONLY — makes limits '
                        'active with motors off so you can hand-test triggers)')
    p.add_argument('--dry-run', action='store_true',
                   help='log violations and intended actions without sending commands')
    p.add_argument('--status', action='store_true',
                   help='print a 1 Hz telemetry line (auto-on in dry-run)')
    args = p.parse_args()
    if args.land_speed <= 0:
        p.error('--land-speed must be > 0')

    uri = uri_helper.uri_from_env(default=args.uri)
    cflib.crtp.init_drivers()
    print(f'Connecting to {uri} ...')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='/tmp/cflib_cache')) as scf:
        wd = SafetyWatchdog(scf.cf, args)
        wd.start_logging()
        print(f'Safety watchdog armed{" [DRY RUN]" if args.dry_run else ""} '
              f'(mocap-free): tilt ≤ {args.tilt_max}° (kill {args.tilt_kill}°), '
              f'rate ≤ {args.rate_max}°/s (kill {args.rate_kill}°/s), '
              f'baro ceiling {args.z_max} m (kill {args.z_kill} m), '
              f'thrust sat {args.thrust_sat * 100:.0f}%, '
              f'telemetry timeout {args.timeout * 1000:.0f} ms. Ctrl+C to land.')
        try:
            while wd.check():
                time.sleep(0.05)
        except KeyboardInterrupt:
            try:
                wd.request_land()
                while wd.state != DONE and wd.check():
                    time.sleep(0.05)
            except KeyboardInterrupt:
                wd.emergency('second Ctrl+C')
        time.sleep(0.3)   # let queued packets go out before the link closes


if __name__ == '__main__':
    main()
