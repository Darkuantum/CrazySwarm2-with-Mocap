#!/usr/bin/env python3
"""RViz markers for the escort: the VIP, the ring, and what each drone was TOLD.

Why this exists
---------------
In sim the picture is only the drones' own ``/tf``. The VIP -- the whole thing
the demo is about -- is a number inside the script and appears nowhere, so a
ring tightening on one side is indistinguishable from the ring drifting, and
"moving to intercept" looks exactly like "moving to follow". This publishes
the missing half.

Everything here is drawn from the COMMANDED setpoints, not from where the
drones ended up. That is deliberate: the gap between the two is the controller
being fought by the guard or by the physics, and it is worth seeing. In sim
the drones track their setpoints almost exactly and the two coincide; on
hardware they will not, and the trail is then the honest record of what the
script asked for.

Nothing in here may break a flight. Every publish is wrapped -- a marker that
fails to serialise must cost a picture, never a drone.

    ros2 run crazyflie_shows escort_show --ros-args -p use_sim_time:=true \\
        -p vip_mode:=point -p check_placement:=False
    # then add a MarkerArray display on /escort/markers (config.rviz has one)
"""

from collections import deque

import numpy as np

try:                                            # keeps the module importable
    from builtin_interfaces.msg import Duration  # for offline tests without ROS
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    _HAVE_ROS = True
except ImportError:                             # pragma: no cover
    _HAVE_ROS = False

TOPIC = '/escort/markers'
FRAME = 'world'

#: Matches the LED palette in escort_show, so the picture on screen and the
#: lights on the drones say the same thing at the same moment.
C_VIP = (1.00, 1.00, 1.00, 0.95)      # white -- the thing being protected
C_ADVERSARY = (1.00, 0.15, 0.10, 0.95)  # red
C_BLOCKING = (1.00, 0.65, 0.00, 0.95)   # amber -- the lead, on the threat bearing
C_WING = (0.65, 0.30, 0.95, 0.95)       # violet -- closed up beside the lead
C_DEFENDER = (0.20, 0.45, 1.00, 0.95)   # deep blue -- resting ring
C_RING = (0.35, 0.55, 0.85, 0.35)       # the commanded ring itself
C_THREAT = (1.00, 0.35, 0.15, 0.80)     # VIP -> adversary bearing
C_VIP_TRAIL = (0.90, 0.90, 0.90, 0.55)  # where the VIP has been
C_ADV_TRAIL = (1.00, 0.35, 0.30, 0.55)  # where the adversary has been
#: The adversary's PLANNED path, drawn ahead of it. Only exists when something
#: generates the path -- a script, or a reactive law. Under human control there
#: is no plan to draw, and the trail is the whole truth.
C_ADV_PLAN = (1.00, 0.55, 0.30, 0.40)

#: How many commanded points a trail keeps. 150 at 10 Hz is 15 s -- long
#: enough to show a ring rotation (a 120 deg swing takes ~7 s) and short
#: enough that the picture does not turn into a ball of wool.
TRAIL_LEN = 150


class EscortViz:
    """Publishes one MarkerArray per tick. Safe to construct when ROS is absent."""

    def __init__(self, node, n_defenders, names=None, every=2):
        """``every``: publish on 1 of N control steps (20 Hz / 2 = 10 Hz)."""
        self.ok = _HAVE_ROS and node is not None
        self.every = max(1, int(every))
        self.k = 0
        self.names = list(names or [])
        self.trails = [deque(maxlen=TRAIL_LEN) for _ in range(n_defenders)]
        self.vip_trail = deque(maxlen=TRAIL_LEN)
        self.adv_trail = deque(maxlen=TRAIL_LEN)
        self.pub = None
        if self.ok:
            try:
                self.pub = node.create_publisher(MarkerArray, TOPIC, 1)
            except Exception:                   # noqa: BLE001
                self.ok = False

    # -- marker helpers ---------------------------------------------------
    def _base(self, mid, kind, scale, colour, ns='escort'):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = ns
        m.id = int(mid)
        m.type = kind
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color = ColorRGBA(r=float(colour[0]), g=float(colour[1]),
                            b=float(colour[2]), a=float(colour[3]))
        m.pose.orientation.w = 1.0
        # Markers expire if the script dies mid-flight, so a stale picture
        # cannot be mistaken for a live one.
        m.lifetime = Duration(sec=1, nanosec=0)
        return m

    def _sphere(self, mid, p, d, colour):
        m = self._base(mid, Marker.SPHERE, (d, d, d), colour)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            float(p[0]), float(p[1]), float(p[2]))
        return m

    def _line(self, mid, pts, width, colour, strip=True):
        m = self._base(mid, Marker.LINE_STRIP if strip else Marker.LINE_LIST,
                       (width, 0.0, 0.0), colour)
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        return m

    def _text(self, mid, p, text, colour, size=0.18):
        m = self._base(mid, Marker.TEXT_VIEW_FACING, (0.0, 0.0, size), colour)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            float(p[0]), float(p[1]), float(p[2]) + 0.22)
        m.text = str(text)
        return m

    # -- the one call the flight loop makes -------------------------------
    def publish(self, p_vip, p_adv, setpoints, info, ring_radius, force=False,
                adv_plan=None):
        """Draw the current state. Never raises.

        ``adv_plan``: the adversary's future path as a sequence of points, when
        something generates one (a script, or a reactive law). Pass None under
        human control -- there is no plan, and inventing a line would claim the
        demo knows where a person is about to fly.
        """
        if not self.ok or self.pub is None:
            return
        self.k += 1
        if not force and self.k % self.every:
            return
        try:
            self._publish(p_vip, p_adv, setpoints, info, ring_radius, adv_plan)
        except Exception:                        # noqa: BLE001
            self.ok = False                      # one failure, then stay quiet

    def _publish(self, p_vip, p_adv, setpoints, info, ring_radius, adv_plan=None):
        arr = MarkerArray()
        engaged = bool(info.get('engaged'))
        lead = info.get('lead', 0)
        slot_of = info.get('slot_of')            # optional callable or sequence

        vip = np.asarray(p_vip, float)
        arr.markers.append(self._sphere(0, vip, 0.26, C_VIP))
        arr.markers.append(self._text(1, vip, 'VIP', C_VIP))
        self.vip_trail.append(vip.copy())
        if len(self.vip_trail) > 1:
            arr.markers.append(self._line(6, self.vip_trail, 0.012, C_VIP_TRAIL))

        # the ring the defenders are being held on, drawn where it is commanded
        circle = [(vip[0] + ring_radius * np.cos(a),
                   vip[1] + ring_radius * np.sin(a),
                   float(setpoints[0][2]) if len(setpoints) else vip[2])
                  for a in np.linspace(0, 2 * np.pi, 49)]
        arr.markers.append(self._line(2, circle, 0.012, C_RING))

        if p_adv is not None:
            adv = np.asarray(p_adv, float)
            arr.markers.append(self._sphere(3, adv, 0.22, C_ADVERSARY))
            arr.markers.append(self._text(4, adv, 'adversary', C_ADVERSARY))
            self.adv_trail.append(adv.copy())
            if len(self.adv_trail) > 1:
                arr.markers.append(self._line(7, self.adv_trail, 0.014, C_ADV_TRAIL))
            if adv_plan is not None and len(adv_plan) > 1:
                arr.markers.append(self._line(8, adv_plan, 0.010, C_ADV_PLAN))
            if engaged:
                # the bearing the ring is answering -- the line that explains
                # why the formation turned the way it did
                arr.markers.append(self._line(5, [vip, adv], 0.02, C_THREAT,
                                              strip=False))

        for i, sp in enumerate(setpoints):
            if slot_of is None:
                is_lead = (i == lead)
            elif callable(slot_of):
                is_lead = (slot_of(i) == lead)
            else:
                is_lead = (slot_of[i] == lead)
            colour = (C_BLOCKING if (engaged and is_lead)
                      else C_WING if engaged else C_DEFENDER)
            self.trails[i].append(np.asarray(sp, float).copy())
            arr.markers.append(self._sphere(10 + i, sp, 0.13, colour))
            if len(self.trails[i]) > 1:
                arr.markers.append(self._line(30 + i, self.trails[i], 0.01, colour))
            if i < len(self.names):
                arr.markers.append(self._text(50 + i, sp, self.names[i], colour, 0.12))

        self.pub.publish(arr)
