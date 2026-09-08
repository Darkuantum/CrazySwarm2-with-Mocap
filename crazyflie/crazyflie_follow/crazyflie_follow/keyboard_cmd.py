#!/usr/bin/env python3
"""Keyboard stand-in for the tablet/gesture-band command channel.

follow_node runs a follow_core.Follower whose CommandArbiter treats a command
as expired once it is older than `command_timeout_s` (2.0 s default) --
"a command from a channel that has gone quiet is not a command", per
arbiter.py's own docstring. A real tablet or wrist band is expected to keep
asserting its currently-selected mode continuously; a single keypress is not
that. So this node republishes the operator's current mode on a timer faster
than the arbiter's timeout, rather than only on a keypress.

That is deliberate, not incidental: it means killing this process (Ctrl-C,
laptop closed, Wi-Fi drops) reads to the arbiter exactly like a dead tablet --
the FOLLOW/ORBIT assertion goes stale and follow_node's Follower falls back to
HOLD within `command_timeout_s` on its own, with no special-casing needed
here for "the operator's controller disappeared". That fallback is the
intended fail-safe.

ESTOP is different by construction (arbiter.py): it LATCHES on the first
press and does not need to be repeated, and nothing but an explicit
`clear_estop` releases it -- mirroring a physical E-stop that stays down
until someone chooses to bring it back up. So pressing space here is enough;
it does not need the heartbeat, though the heartbeat harmlessly keeps
resending it anyway.

Keys (non-blocking, via the same termios/select KeyPoller crazyflie_py
already ships for its own teleop -- see crazyflie_py/keyboard.py -- so this
does not add a curses dependency):

    f       FOLLOW
    s       HOLD          ("stop" -- keep hovering in place)
    o       ORBIT
    l       LAND
    space   ESTOP         latches immediately; nothing here can clear it but 'c'
    c       clear ESTOP   explicit operator action, sent once
    q       quit this process (does NOT itself command a mode change -- the
            vehicle's last-heartbeated mode simply goes stale and follow_node
            falls back to HOLD, per the module docstring above)

Run:
    ros2 run crazyflie_follow keyboard_cmd
    ros2 run crazyflie_follow keyboard_cmd --ros-args -p command_topic:=/follow/cmd
"""
import rclpy
from crazyflie_py.keyboard import KeyPoller
from rclpy.node import Node
from std_msgs.msg import String

#: Well inside follow_core's 2.0 s command_timeout_s / 1.5 s estop_timeout_s
#: defaults -- see arbiter.py -- with comfortable margin for a dropped packet
#: or two.
HEARTBEAT_HZ = 5.0
HEARTBEAT_PERIOD_S = 1.0 / HEARTBEAT_HZ

#: How often to poll the keyboard. Short relative to the heartbeat period so
#: a keypress (especially ESTOP) is published close to immediately rather
#: than waiting out the rest of a heartbeat tick.
POLL_PERIOD_S = 0.05

_KEY_TOKEN = {
    'f': 'follow',
    's': 'hold',
    'o': 'orbit',
    'l': 'land',
    ' ': 'estop',
}

_DEFAULT_MODE = 'hold'   # matches the arbiter's own "no fresh command" fallback


def main(args=None):
    rclpy.init(args=args)
    node = Node('keyboard_cmd')
    node.declare_parameter('command_topic', '/follow/cmd')
    topic = node.get_parameter('command_topic').value
    pub = node.create_publisher(String, topic, 10)

    print(__doc__)
    print(f'publishing to {topic!r}; current mode: {_DEFAULT_MODE}\n')

    current = _DEFAULT_MODE
    last_beat = node.get_clock().now().nanoseconds * 1e-9 - HEARTBEAT_PERIOD_S

    with KeyPoller() as poller:
        try:
            while rclpy.ok():
                key = poller.poll()
                if key is not None:
                    if key == 'q':
                        break
                    if key == 'c':
                        pub.publish(String(data='clear_estop'))
                        print('-> clear_estop')
                    elif key in _KEY_TOKEN:
                        current = _KEY_TOKEN[key]
                        pub.publish(String(data=current))
                        print(f'-> {current}')
                    # any other key is ignored, not an error -- a stray
                    # keystroke on this console must never look like a
                    # vehicle command

                now = node.get_clock().now().nanoseconds * 1e-9
                if now - last_beat >= HEARTBEAT_PERIOD_S:
                    pub.publish(String(data=current))
                    last_beat = now

                rclpy.spin_once(node, timeout_sec=POLL_PERIOD_S)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
