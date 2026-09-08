"""Closed-loop follower simulation -- scores the stack against the PRD.

Nothing here touches hardware. It flies a simple vehicle model against a
synthetic walking operator, feeding the real `follow_core.Follower` through
noisy UWB and vision models, and reports whether FR-1's acceptance criteria are
met. Run it before every flight session; if a change breaks tracking here it
will break it in the volume too.

Sensor models use the published figures the design was chosen against:
  UWB   ~5 Hz two-way ranging, sigma 6 cm, plus a systematic bias and the
        occasional multipath outlier the literature documents.
  Vision 30 Hz bearing, sigma ~0.5 deg, dropping out entirely when the operator
        is occluded or leaves the field of view.

Usage:
    python3 tools/sim.py                     # default scenario, prints a report
    python3 tools/sim.py --scenario occlude  # walk behind a pillar
    python3 tools/sim.py --plot out.png      # optional, needs matplotlib
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from follow_core import (  # noqa: E402
    BearingMeasurement,
    Command,
    CommandSource,
    FollowMode,
    FollowParams,
    Follower,
    RangeMeasurement,
    TrackHealth,
    VehicleState,
)
from follow_core.ekf import _bearing_from_vector  # noqa: E402

FOV_H = math.radians(65.0)   # AR0144-class horizontal field of view
FOV_V = math.radians(50.0)
# Camera down-pitch. This is a real design constraint the simulation surfaced:
# at standoff 2.5 m and height 2.0 m the operator sits ~39 deg below horizontal,
# which is outside a 50 deg vertical FOV unless the camera is pitched down. At
# 15 deg the operator rides the bottom edge of the frame and any disturbance
# drops the bearing measurement entirely.
CAM_PITCH_DEG = 25.0


# -- operator trajectories ---------------------------------------------------

def walk_straight_then_turn(t: float) -> np.ndarray:
    """1.2 m/s straight, a true 90-degree turn, then straight again.

    The arc radius is chosen so the turn closes on exactly pi/2 at constant
    speed -- an arc that does not close leaves a position step at the seam,
    which shows up as an impossible operator speed and is a bug in the test,
    not in the follower.
    """
    v, turn_t = 1.2, 4.0
    r = v * turn_t / (math.pi / 2.0)      # arc length = v*turn_t = r*pi/2
    if t < 8.0:
        return np.array([v * t, 0.0, 0.0])
    if t < 8.0 + turn_t:
        a = (math.pi / 2.0) * (t - 8.0) / turn_t
        return np.array([9.6 + r * math.sin(a), r * (1.0 - math.cos(a)), 0.0])
    return np.array([9.6 + r, r + v * (t - 8.0 - turn_t), 0.0])


def walk_figure_eight(t: float) -> np.ndarray:
    a, w = 4.0, 0.25
    return np.array([a * math.sin(w * t), a * math.sin(w * t) * math.cos(w * t), 0.0])


def walk_fast(t: float) -> np.ndarray:
    """1.4 m/s -- the upper bound FR-1 commits to."""
    return np.array([1.4 * t, 0.6 * math.sin(0.35 * t), 0.0])


SCENARIOS = {
    "straight": walk_straight_then_turn,
    "eight": walk_figure_eight,
    "fast": walk_fast,
    "occlude": walk_straight_then_turn,
}


@dataclass
class SimResult:
    t: np.ndarray
    standoff_err: np.ndarray
    person: np.ndarray
    drone: np.ndarray
    est: np.ndarray
    health: List[str] = field(default_factory=list)
    modes: List[str] = field(default_factory=list)
    range_rejected: int = 0
    bearing_rejected: int = 0
    min_separation: float = 0.0
    lost_fraction: float = 0.0
    vis_fraction: float = 0.0

    @property
    def mean_err(self) -> float:
        return float(np.mean(self.standoff_err))

    @property
    def p95_err(self) -> float:
        return float(np.percentile(self.standoff_err, 95))

    def report(self, params: FollowParams) -> str:
        ok_mean = self.mean_err <= 0.35
        ok_p95 = self.p95_err <= 0.75
        ok_sep = self.min_separation >= params.min_standoff_m - 0.15
        mark = lambda b: "PASS" if b else "FAIL"  # noqa: E731
        return "\n".join(
            [
                "",
                "  FR-1 acceptance (mocap-referenced following)",
                "  " + "-" * 52,
                f"  mean standoff error   {self.mean_err:6.3f} m   limit 0.350   {mark(ok_mean)}",
                f"  p95  standoff error   {self.p95_err:6.3f} m   limit 0.750   {mark(ok_p95)}",
                "",
                "  Safety",
                "  " + "-" * 52,
                f"  min separation        {self.min_separation:6.3f} m   floor {params.min_standoff_m - 0.15:.3f}   {mark(ok_sep)}",
                f"  time target LOST      {self.lost_fraction * 100:6.1f} %",
                f"  operator in frame     {self.vis_fraction * 100:6.1f} %",
                "",
                "  Estimator",
                "  " + "-" * 52,
                f"  range   rejected      {self.range_rejected}",
                f"  bearing rejected      {self.bearing_rejected}",
                "",
                f"  OVERALL: {mark(ok_mean and ok_p95 and ok_sep)}",
                "",
            ]
        )

    @property
    def passed(self) -> bool:
        return self.mean_err <= 0.35 and self.p95_err <= 0.75


def run_sim(
    scenario: str = "straight",
    duration: float = 20.0,
    dt: float = 0.02,
    params: Optional[FollowParams] = None,
    seed: int = 42,
    occlusions: Optional[List[tuple]] = None,
) -> SimResult:
    p = params or FollowParams()
    rng = np.random.default_rng(seed)
    path: Callable[[float], np.ndarray] = SCENARIOS[scenario]

    if occlusions is None:
        occlusions = [(6.0, 8.5), (13.0, 14.2)] if scenario == "occlude" else []

    f = Follower(p)

    drone = np.array([-p.standoff_m, 0.0, p.height_m])
    dvel = np.zeros(3)
    yaw = 0.0
    uwb_bias = rng.normal(0.0, 0.04)

    n = int(duration / dt)
    ts, errs, ppos, dpos, epos, health, modes = [], [], [], [], [], [], []
    next_uwb, next_vis, next_cmd = 0.0, 0.0, 0.0
    lost_ticks = 0
    min_sep = 1e9
    vis_attempts = vis_ok = 0

    for k in range(n):
        t = k * dt
        person = path(t)
        pv = (path(t + 1e-3) - path(t - 1e-3)) / 2e-3 if t > 0 else np.zeros(3)

        # Camera looks along the vehicle yaw, pitched 15 deg down.
        pitch = math.radians(CAM_PITCH_DEG)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp_ = math.cos(pitch), math.sin(pitch)
        # world <- camera, for an optical frame (X right, Y down, Z forward)
        fwd = np.array([cy * cp, sy * cp, -sp_])
        right = np.array([sy, -cy, 0.0])
        down = np.cross(fwd, right)
        R_wc = np.column_stack([right, down, fwd])

        f.on_vehicle(VehicleState(stamp=t, position=drone.copy(), velocity=dvel.copy(),
                                  yaw=yaw, R_world_cam=R_wc))

        occluded = any(a <= t <= b for a, b in occlusions)

        # -- UWB: survives occlusion, that is the whole point of carrying it --
        if t >= next_uwb:
            next_uwb = t + 1.0 / 5.0
            true_r = float(np.linalg.norm(person - drone))
            z = true_r + uwb_bias + rng.normal(0.0, 0.06)
            if rng.random() < 0.02:            # documented multipath outlier
                z += rng.uniform(1.5, 6.0)
            f.on_range(RangeMeasurement(stamp=t, range_m=float(z), tag_id=7, std_m=0.08))

        # -- Vision: high rate, but only with line of sight and in frame ------
        if t >= next_vis:
            next_vis = t + 1.0 / 30.0
            d_cam = R_wc.T @ (person - drone)
            vis_attempts += 1
            if d_cam[2] > 0.3 and not occluded:
                az, el = _bearing_from_vector(d_cam)
                if abs(az) < FOV_H / 2 and abs(el) < FOV_V / 2:
                    vis_ok += 1
                    f.on_bearing(
                        BearingMeasurement(
                            stamp=t,
                            azimuth_rad=az + rng.normal(0.0, 0.009),
                            elevation_rad=el + rng.normal(0.0, 0.009),
                            std_rad=0.01,
                            score=0.9,
                        )
                    )

        # The tablet re-asserts its command at 2 Hz. This is not decoration:
        # the arbiter deliberately expires a command older than
        # command_timeout_s, so a ground station that stops talking must not
        # leave the vehicle in FOLLOW. Any real command source needs a
        # heartbeat -- reproduce that here or the sim is not testing reality.
        if t >= next_cmd:
            next_cmd = t + 0.5
            f.on_command(Command(mode=FollowMode.FOLLOW, source=CommandSource.TABLET, stamp=t))

        out = f.step(t)
        sp = out.setpoint

        # -- vehicle model: velocity command with a first-order lag ----------
        err = sp.position - drone
        v_cmd = 1.6 * err + sp.velocity_ff
        s = float(np.linalg.norm(v_cmd))
        if s > sp.max_speed > 0.0:
            v_cmd *= sp.max_speed / s
        elif sp.max_speed <= 0.0:
            v_cmd = np.zeros(3)
        tau = 0.30
        dvel += (v_cmd - dvel) * (dt / tau)
        drone = drone + dvel * dt
        yaw += (math.atan2(math.sin(sp.yaw - yaw), math.cos(sp.yaw - yaw))) * min(1.0, dt / 0.25)

        sep = float(np.linalg.norm(drone[:2] - person[:2]))
        min_sep = min(min_sep, sep)
        if out.health is TrackHealth.LOST:
            lost_ticks += 1

        # Score only once the filter has settled -- the first second is startup.
        if t >= 2.0:
            ts.append(t)
            errs.append(abs(sep - p.standoff_m))
            ppos.append(person.copy())
            dpos.append(drone.copy())
            epos.append(out.estimate.position.copy() if out.estimate else drone.copy())
            health.append(out.health.value)
            modes.append(out.mode.value)

    return SimResult(
        t=np.array(ts),
        standoff_err=np.array(errs),
        person=np.array(ppos),
        drone=np.array(dpos),
        est=np.array(epos),
        health=health,
        modes=modes,
        range_rejected=f.ekf.range_rejected,
        bearing_rejected=f.ekf.bearing_rejected,
        min_separation=min_sep,
        lost_fraction=lost_ticks / n,
        vis_fraction=(vis_ok / vis_attempts) if vis_attempts else 0.0,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Closed-loop follower simulation")
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="straight")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--standoff", type=float, default=None)
    ap.add_argument("--plot", type=str, default=None)
    a = ap.parse_args()

    p = FollowParams()
    if a.standoff is not None:
        p.standoff_m = a.standoff
    p.validate()

    r = run_sim(scenario=a.scenario, duration=a.duration, params=p, seed=a.seed)
    print(f"\nscenario: {a.scenario}   duration: {a.duration:.0f} s   standoff: {p.standoff_m:.2f} m")
    print(r.report(p))

    if a.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed -- skipping plot")
            return 0 if r.passed else 1
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        ax[0].plot(r.person[:, 0], r.person[:, 1], label="operator", lw=2)
        ax[0].plot(r.drone[:, 0], r.drone[:, 1], label="drone", lw=1.5)
        ax[0].plot(r.est[:, 0], r.est[:, 1], "--", label="estimate", lw=1)
        ax[0].set_aspect("equal"); ax[0].legend(); ax[0].set_title("ground track")
        ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].grid(alpha=.3)
        ax[1].plot(r.t, r.standoff_err)
        ax[1].axhline(0.35, ls="--", c="C1", label="mean limit")
        ax[1].axhline(0.75, ls=":", c="C3", label="p95 limit")
        ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("standoff error [m]")
        ax[1].legend(); ax[1].grid(alpha=.3); ax[1].set_title("tracking error")
        fig.tight_layout(); fig.savefig(a.plot, dpi=130)
        print(f"  plot written to {a.plot}")

    return 0 if r.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
