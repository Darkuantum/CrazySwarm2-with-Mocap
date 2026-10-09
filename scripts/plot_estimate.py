#!/usr/bin/env python3
"""Live plot of stateEstimate.x/y/z (+ yaw) from crazyswarm2.

Subscribes to /<drone>/estimator (crazyflie_interfaces/LogDataGeneric), whose
`values` are [stateEstimate.x, stateEstimate.y, stateEstimate.z, stateEstimate.yaw]
as defined in crazyflies.yaml -> all.firmware_logging.custom_topics.estimator.
(yaw is optional: if the yaml still lists only x/y/z, the yaw axis stays empty.)

Run the crazyflie stack first (backend:=cpp or cflib) so the drone is logging,
then (PYTHONNOUSERSITE=1 avoids the ~/.local numpy 2.x vs apt-matplotlib clash):

    PYTHONNOUSERSITE=1 python3 plot_estimate.py                 # defaults to cf2
    PYTHONNOUSERSITE=1 python3 plot_estimate.py cf1 cf2         # one subplot per drone
    PYTHONNOUSERSITE=1 python3 plot_estimate.py --window 30 cf2 # 30 s of history

With mocap feeding /poses, x/y/z should lock flat at the true position. Yaw (right
axis, degrees) should hold constant; a slow ramp/offset on one drone is the
single-marker heading problem that makes it shoot off sideways on takeoff.
"""
import argparse
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

POS = ('x', 'y', 'z')       # values[0:3], metres, left axis
YAW = 'yaw'                 # values[3],   degrees, right axis


class EstimateListener(Node):
    def __init__(self, drones, window):
        super().__init__('plot_estimate')
        self.window = window
        self.lock = threading.Lock()
        self.t0 = None
        self.data = {n: {k: deque() for k in ('t',) + POS + (YAW,)} for n in drones}
        for name in drones:
            self.create_subscription(
                LogDataGeneric, f'/{name}/estimator',
                lambda msg, n=name: self.cb(msg, n), 10)
        self.get_logger().info(f'Listening for /<drone>/estimator on: {drones}')

    def cb(self, msg, name):
        if len(msg.values) < 3:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            d = self.data[name]
            d['t'].append(t - self.t0)
            for i, a in enumerate(POS):
                d[a].append(msg.values[i])
            # yaw is values[3] once added to the yaml; NaN keeps the deques aligned
            d[YAW].append(msg.values[3] if len(msg.values) > 3 else float('nan'))
            while d['t'] and d['t'][-1] - d['t'][0] > self.window:
                for k in ('t',) + POS + (YAW,):
                    d[k].popleft()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('drones', nargs='*', help='drone names (default: cf2)')
    parser.add_argument('--window', type=float, default=20.0,
                        help='seconds of history to show (default: 20)')
    args = parser.parse_args()
    drones = args.drones or ['cf2']

    rclpy.init()
    node = EstimateListener(drones, args.window)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, axarr = plt.subplots(len(drones), 1, sharex=True, squeeze=False)
    axarr = axarr[:, 0]
    lines, yaw_axes = {}, {}
    for ax, name in zip(axarr, drones):
        lines[name] = {a: ax.plot([], [], label=a)[0] for a in POS}
        ax.set_ylabel(f'{name}\nx/y/z [m]')
        ax.grid(True)
        ax2 = ax.twinx()                       # right axis for yaw (degrees)
        lines[name][YAW] = ax2.plot([], [], color='red', ls='--', label='yaw')[0]
        ax2.set_ylabel('yaw [deg]', color='red')
        ax2.tick_params(axis='y', labelcolor='red')
        yaw_axes[name] = ax2
        h = [lines[name][a] for a in POS] + [lines[name][YAW]]
        ax.legend(h, [ln.get_label() for ln in h], loc='upper left', ncol=4)
    axarr[-1].set_xlabel('time [s]')
    fig.suptitle('crazyswarm2 state estimate (live)')

    def update(_):
        with node.lock:
            for ax, name in zip(axarr, drones):
                t = list(node.data[name]['t'])
                for a in POS + (YAW,):
                    lines[name][a].set_data(t, list(node.data[name][a]))
                if t:
                    ax.set_xlim(max(0.0, t[-1] - node.window), max(t[-1], node.window))
                    for a in (ax, yaw_axes[name]):
                        a.relim()
                        a.autoscale_view(scalex=False, scaley=True)
        return [ln for name in drones for ln in lines[name].values()]

    # keep a reference: an unassigned FuncAnimation is garbage-collected before
    # plt.show() renders, leaving the axes empty
    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
