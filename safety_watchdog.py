#!/usr/bin/env python3
"""MOCAP-FREE safety watchdog for crazyswarm2 — runaway protection over ROS 2.

Unlike mocap_watchdog.py (which watches the /poses feed) and unlike a position
geofence, this node NEVER trusts position. With estimator=2 the onboard estimate
stateEstimate.x/y/z is fused with the mocap pose the server sends, so it inherits
mocap frame drift — worthless for a safety fence exactly when mocap goes bad.

Instead it watches only mocap-free onboard signals, streamed by the firmware as
the safety_att / safety_sys log topics (see crazyflies.yaml custom_topics):

  * tilt         stabilizer.roll / pitch   (attitude est: gyro+accel)
  * angular rate gyro.x / gyro.y / gyro.z  (raw rate gyro)
  * tumble/crash supervisor.info bits       (firmware's own verdict)
  * thrust       stabilizer.thrust          (controller flooring throttle)
  * altitude     baro.asl                   (barometric, vs. takeoff ref)
  * link/battery topic timing / pm.vbat

A runaway from mocap drift shows up here as violent behaviour (the controller
chases a bad setpoint), which is mocap-free and catchable; the position drift
itself is not, by anyone, without an independent position sensor.

Escalation while airborne:
  * SOFT limit (tilt/rate/thrust/baro-ceiling/battery) -> controlled land
                straight down (no go_to: without position we cannot hold xy),
                then disarm.
  * HARD limit (tumble, crash, extreme tilt/rate, hard ceiling, telemetry gone)
                -> immediate motor kill (/cfX/emergency).

Run alongside the stack (rebuild crazyflie first so the new log topics exist):

    python3 safety_watchdog.py                       # defaults, drone cf3
    python3 safety_watchdog.py --drone cf1 --tilt-max 40
    python3 safety_watchdog.py --dry-run --assume-flying   # bench (props off)

NOTE: horizontal position is deliberately NOT fenced — see module docstring of
safety_watchdog_cflib.py. This guards against violent runaways, not slow drift.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.srv import Land, Arm, NotifySetpointsStop
from std_srvs.srv import Empty

MONITOR, LANDING, DONE = 'MONITOR', 'LANDING', 'DONE'

# supervisor.info bit positions (cflib/crazyflie/supervisor.py)
BIT_IS_FLYING = 4
BIT_IS_TUMBLED = 5
BIT_IS_CRASHED = 7


class SafetyWatchdog(Node):
    def __init__(self, args):
        super().__init__('safety_watchdog')
        self.args = args
        prefix = f'/{args.drone}'

        self.att = None            # (roll, pitch) deg
        self.rate = None           # (gx, gy, gz) deg/s
        self.thrust = 0.0          # 0..65535
        self.asl = None            # barometric altitude, m
        self.asl_ref = None        # takeoff reference, m
        self.vbat = None
        self.info = 0              # supervisor bitfield
        self.last_att = None       # time of last safety_att message
        self.last_status = None
        self.low_batt_since = None
        self.thrust_sat_since = None
        self.flying = False
        self.state = MONITOR
        self.state_since = None
        self.land_deadline = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LogDataGeneric, f'{prefix}/safety_att', self.on_att, qos)
        self.create_subscription(
            LogDataGeneric, f'{prefix}/safety_sys', self.on_sys, qos)

        self.cli_stop_setpoints = self.create_client(
            NotifySetpointsStop, f'{prefix}/notify_setpoints_stop')
        self.cli_land = self.create_client(Land, f'{prefix}/land')
        self.cli_arm = self.create_client(Arm, f'{prefix}/arm')
        self.cli_emergency = self.create_client(Empty, f'{prefix}/emergency')

        self.create_timer(0.05, self.check)   # 20 Hz
        self.get_logger().info(
            f'Safety watchdog armed on {args.drone}'
            f'{" [DRY RUN]" if args.dry_run else ""} (mocap-free): '
            f'tilt ≤ {args.tilt_max}° (kill {args.tilt_kill}°), '
            f'rate ≤ {args.rate_max}°/s (kill {args.rate_kill}°/s), '
            f'baro ceiling {args.z_max} m (kill {args.z_kill} m), '
            f'thrust sat {args.thrust_sat * 100:.0f}%, '
            f'telemetry timeout {args.timeout * 1000:.0f} ms')

    # ---------------------------------------------------------------- inputs

    def on_att(self, msg):
        if len(msg.values) < 5:
            return
        self.att = (msg.values[0], msg.values[1])
        self.rate = (msg.values[2], msg.values[3], msg.values[4])
        self.last_att = self.get_clock().now()

    def on_sys(self, msg):
        if len(msg.values) < 4:
            return
        self.thrust = msg.values[0]
        self.asl = msg.values[1]
        self.vbat = msg.values[2]
        self.info = int(round(msg.values[3]))
        if self.asl_ref is None:
            self.asl_ref = self.asl

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
            self.get_logger().info(
                'Airborne — limits active' if self.flying else 'On ground')
        if not self.flying:
            self.asl_ref = self.asl
            self.low_batt_since = None
            self.thrust_sat_since = None

    # ------------------------------------------------------------ monitoring

    def _maybe_status(self, now):
        if not (self.args.status or self.args.dry_run) or self.att is None:
            return
        if self.last_status is not None and \
                (now - self.last_status).nanoseconds * 1e-9 < 1.0:
            return
        self.last_status = now
        roll, pitch = self.att
        rate = max(abs(r) for r in self.rate) if self.rate else 0.0
        self.get_logger().info(
            f'[status] tilt=({roll:+.0f},{pitch:+.0f})° rate={rate:.0f}°/s '
            f'thr={self.thrust / 65535 * 100:.0f}% alt={self.rel_alt:+.2f}m '
            f'flying={self.flying} tumbled={self._bit(BIT_IS_TUMBLED)}')

    def check(self):
        if self.state == DONE:
            return
        now = self.get_clock().now()
        self.update_flying()
        self._maybe_status(now)

        if self.flying and self.last_att is not None:
            gap = (now - self.last_att).nanoseconds * 1e-9
            if gap > self.args.timeout:
                self.emergency(f'telemetry lost for {gap * 1000:.0f} ms mid-flight')
                return

        if not self.flying or self.att is None:
            return

        hard = self.hard_violation()
        if hard:
            self.emergency(hard)
            return

        if self.state == MONITOR:
            soft = self.soft_violation(now)
            if soft:
                self.get_logger().error(f'LIMIT EXCEEDED: {soft} -> landing')
                self.start_landing(now)
        elif self.state == LANDING:
            self.continue_landing(now)

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
        if self.thrust / 65535.0 > a.thrust_sat:
            if self.thrust_sat_since is None:
                self.thrust_sat_since = now
            elif (now - self.thrust_sat_since).nanoseconds * 1e-9 > a.thrust_sat_time:
                return (f'thrust saturated {self.thrust / 65535 * 100:.0f}% for '
                        f'{a.thrust_sat_time:.0f} s')
        else:
            self.thrust_sat_since = None
        if self.vbat is not None and self.vbat < a.vbat_land:
            if self.low_batt_since is None:
                self.low_batt_since = now
            elif (now - self.low_batt_since).nanoseconds * 1e-9 > 2.0:
                return f'battery {self.vbat:.2f} V < {a.vbat_land} V for 2 s'
        else:
            self.low_batt_since = None
        return None

    # ------------------------------------------------------------- responses

    def start_landing(self, now):
        duration = max(self.args.land_min_duration,
                       max(self.rel_alt, 0.0) / self.args.land_speed)
        self.state, self.state_since = LANDING, now
        self.land_deadline = now + Duration(seconds=duration + 2.0)
        self.get_logger().error(
            f'LANDING from {self.rel_alt:.2f} m, {duration:.1f} s descent')
        if self.args.dry_run:
            self.get_logger().warn('[dry-run] would notify_setpoints_stop + land')
            return
        if self.cli_stop_setpoints.service_is_ready():
            req = NotifySetpointsStop.Request()
            req.remain_valid_millisecs = 0
            req.group_mask = 0
            self.cli_stop_setpoints.call_async(req)
        req = Land.Request()
        req.group_mask = 0
        req.height = self.args.land_height
        req.duration = Duration(seconds=duration).to_msg()
        self.call(self.cli_land, req, 'land')

    def continue_landing(self, now):
        on_ground = (not self._bit(BIT_IS_FLYING)
                     and self.rel_alt < self.args.land_height + 0.10)
        if not on_ground and now < self.land_deadline:
            return
        if not on_ground:
            self.emergency(f'still airborne ({self.rel_alt:.2f} m) after land deadline')
            return
        self.state = DONE
        self.get_logger().info('Landed. Disarming.')
        if not self.args.dry_run:
            req = Arm.Request()
            req.arm = False
            self.call(self.cli_arm, req, 'disarm')
        self.get_logger().info('Safety sequence complete — restart node to re-arm')

    def emergency(self, reason):
        self.state = DONE
        self.get_logger().fatal(f'EMERGENCY STOP: {reason} -> cutting motors')
        if self.args.dry_run:
            self.get_logger().warn('[dry-run] would call emergency')
            return
        self.call(self.cli_emergency, Empty.Request(), 'emergency')

    def call(self, client, req, label):
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.5)
        if client.service_is_ready():
            client.call_async(req)
        else:
            self.get_logger().error(f'{label} service unavailable!')


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--drone', default='cf3', help='drone namespace (default cf3)')
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
                   help='land service target height, m (default 0.04)')
    p.add_argument('--land-speed', type=float, default=0.3,
                   help='descent speed used to size the land duration, m/s (default 0.3)')
    p.add_argument('--land-min-duration', type=float, default=2.0,
                   help='minimum land duration, s (default 2)')
    p.add_argument('--assume-flying', action='store_true',
                   help='force airborne state (BENCH TESTING ONLY — makes limits '
                        'active with motors off so you can hand-test triggers)')
    p.add_argument('--dry-run', action='store_true',
                   help='log violations and intended actions without calling any service')
    p.add_argument('--status', action='store_true',
                   help='log a 1 Hz telemetry line (auto-on in dry-run)')
    args = p.parse_args()
    if args.land_speed <= 0:
        p.error('--land-speed must be > 0')

    rclpy.init()
    node = SafetyWatchdog(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
