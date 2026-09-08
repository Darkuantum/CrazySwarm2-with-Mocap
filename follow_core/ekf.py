"""Constant-velocity EKF fusing UWB range with vision bearing.

Why this decomposition: the two sensors fail in opposite places. UWB two-way
ranging gives distance and an unambiguous radio identity that survives
occlusion, but its angle-of-arrival is slow (~4.6 Hz) and only accurate inside
a narrow forward cone. Vision gives sub-degree bearing at 30 Hz but carries no
identity and dies the moment the operator is occluded.

So: take range from UWB, bearing from vision, and let the filter -- anchored by
the range -- decide whether a given detection is even plausibly the operator.
That gating is what implements FR-2 (no lock transfer to a different person).

State is the target in the WORLD frame, not the body frame:

    x = [px, py, pz, vx, vy, vz]

The vehicle moves, so a body-frame target state would have to model the
vehicle's own motion as target motion. In world frame the process model is just
a person walking, and the vehicle pose enters through the measurement model
where it belongs.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .types import (
    BearingMeasurement,
    FollowParams,
    RangeMeasurement,
    TargetEstimate,
    TrackHealth,
    VehicleState,
    wrap_pi,
)

_EPS = 1e-9


def _bearing_from_vector(d_cam: np.ndarray) -> tuple[float, float]:
    """Azimuth/elevation of a vector in the camera optical frame.

    Optical frame is REP-103: X right, Y down, Z forward.
    """
    x, y, z = float(d_cam[0]), float(d_cam[1]), float(d_cam[2])
    rho = float(np.hypot(x, z))
    az = float(np.arctan2(x, z))
    el = float(np.arctan2(-y, rho))
    return az, el


class TargetEKF:
    """Six-state EKF for the operator's position and velocity in world frame."""

    DIM = 6

    def __init__(self, params: Optional[FollowParams] = None) -> None:
        self.p = params or FollowParams()
        self.x = np.zeros(self.DIM)
        self.P = np.eye(self.DIM) * 1e3
        self.stamp: Optional[float] = None
        self.initialized = False
        # Diagnostics -- these are what you look at when following goes wrong.
        self.range_accepted = 0
        self.range_rejected = 0
        self.bearing_accepted = 0
        self.bearing_rejected = 0
        self.last_range_stamp: Optional[float] = None
        self.last_bearing_stamp: Optional[float] = None
        # Consecutive rejections, per sensor. A filter that has diverged rejects
        # everything -- including the measurements that would fix it -- so this
        # is the trip wire for the recovery below.
        self._consec_range_rej = 0
        self._consec_bearing_rej = 0
        self.recoveries = 0

    # -- lifecycle ----------------------------------------------------------

    def initialize(
        self,
        position: np.ndarray,
        stamp: float,
        velocity: Optional[np.ndarray] = None,
        pos_std: float = 1.5,
        vel_std: float = 1.0,
    ) -> None:
        self.x = np.zeros(self.DIM)
        self.x[:3] = np.asarray(position, dtype=float).reshape(3)
        if velocity is not None:
            self.x[3:] = np.asarray(velocity, dtype=float).reshape(3)
        self.P = np.diag(
            [pos_std**2] * 3 + [vel_std**2] * 3
        ).astype(float)
        self.stamp = float(stamp)
        self.initialized = True

    def initialize_from_range_bearing(
        self,
        rng: RangeMeasurement,
        brg: BearingMeasurement,
        vehicle: VehicleState,
    ) -> None:
        """Bootstrap by triangulating a range and a bearing into a position.

        Used on first acquisition and after a LOST->reacquire transition, so the
        filter does not have to converge from a 1000 m^2 prior while the
        operator is walking away.
        """
        # Unit vector toward the target in the camera frame, then to world.
        ca, sa = np.cos(brg.azimuth_rad), np.sin(brg.azimuth_rad)
        ce, se = np.cos(brg.elevation_rad), np.sin(brg.elevation_rad)
        u_cam = np.array([sa * ce, -se, ca * ce], dtype=float)
        u_world = vehicle.R_world_cam @ u_cam
        position = vehicle.position + rng.range_m * u_world
        # Cross-range uncertainty grows with distance; along-range is the UWB std.
        cross = max(rng.range_m * max(brg.std_rad, 1e-3), rng.std_m)
        self.initialize(
            position,
            stamp=max(rng.stamp, brg.stamp),
            pos_std=float(max(cross, rng.std_m)),
            vel_std=1.5,
        )
        self.last_range_stamp = rng.stamp
        self.last_bearing_stamp = brg.stamp

    # -- prediction ---------------------------------------------------------

    def predict(self, stamp: float) -> None:
        """Propagate to `stamp` with a constant-velocity model."""
        if not self.initialized:
            return
        dt = float(stamp) - float(self.stamp)
        if dt <= 0.0:
            # Out-of-order or duplicate sample: hold, do not integrate backwards.
            return
        # Guard against a long gap (e.g. after a dropout) blowing up the state.
        dt = min(dt, 1.0)

        F = np.eye(self.DIM)
        F[:3, 3:] = np.eye(3) * dt

        sa2 = self.p.process_accel_std**2
        # People accelerate far less vertically than horizontally.
        scale = np.array([1.0, 1.0, self.p.vertical_accel_scale])
        q_pp = np.diag(scale * (dt**4) / 4.0)
        q_pv = np.diag(scale * (dt**3) / 2.0)
        q_vv = np.diag(scale * (dt**2))
        Q = np.block([[q_pp, q_pv], [q_pv, q_vv]]) * sa2

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.stamp = float(stamp)

    # -- measurement updates ------------------------------------------------

    def update_range(self, meas: RangeMeasurement, vehicle: VehicleState) -> bool:
        """Fuse a UWB two-way-ranging return. Returns True if accepted."""
        if not self.initialized:
            return False
        self.predict(meas.stamp)

        d = self.x[:3] - vehicle.position
        r = float(np.linalg.norm(d))
        if r < _EPS:
            return False

        H = np.zeros((1, self.DIM))
        H[0, :3] = d / r
        R = np.array([[max(meas.std_m, 1e-3) ** 2]])

        nu = np.array([meas.range_m - r])
        S = H @ self.P @ H.T + R
        nis = float(nu @ np.linalg.solve(S, nu))

        if nis > self.p.range_gate_chi2:
            self.range_rejected += 1
            self._consec_range_rej += 1
            self._maybe_recover()
            return False
        self._consec_range_rej = 0

        # Huber down-weighting. UWB range is documented to carry systematic bias
        # and outliers, so a measurement that passes the gate but sits in the
        # tail gets its influence reduced rather than trusted at face value.
        k = self.p.range_huber_k
        norm_res = float(abs(nu[0]) / np.sqrt(S[0, 0]))
        if norm_res > k:
            R = R * (norm_res / k)
            S = H @ self.P @ H.T + R

        self._apply(H, nu, S)
        self.range_accepted += 1
        self.last_range_stamp = meas.stamp
        return True

    def update_bearing(self, meas: BearingMeasurement, vehicle: VehicleState) -> bool:
        """Fuse a vision bearing. Returns True if accepted.

        Rejection here is the identity mechanism: a detection of a *different*
        person sits at a different bearing than the UWB-anchored estimate
        predicts, fails the chi-square gate, and never reaches the state.
        """
        if not self.initialized:
            return False
        self.predict(meas.stamp)

        R_cw = vehicle.R_world_cam.T
        d_world = self.x[:3] - vehicle.position
        d_cam = R_cw @ d_world
        n2 = float(d_cam @ d_cam)
        if n2 < _EPS:
            return False
        x, y, z = float(d_cam[0]), float(d_cam[1]), float(d_cam[2])
        rho2 = x * x + z * z
        rho = float(np.sqrt(rho2))
        if rho < _EPS:
            return False

        az, el = _bearing_from_vector(d_cam)

        # d(az,el)/d(d_cam)
        J = np.zeros((2, 3))
        J[0, 0] = z / rho2
        J[0, 1] = 0.0
        J[0, 2] = -x / rho2
        J[1, 0] = y * x / (n2 * rho)
        J[1, 1] = -rho / n2
        J[1, 2] = y * z / (n2 * rho)

        H = np.zeros((2, self.DIM))
        H[:, :3] = J @ R_cw

        sd = max(meas.std_rad, 1e-4)
        # A weak detection is a noisier bearing, not a different kind of one.
        infl = 1.0 / max(meas.score, 0.05)
        R = np.eye(2) * (sd**2) * infl

        nu = np.array(
            [wrap_pi(meas.azimuth_rad - az), wrap_pi(meas.elevation_rad - el)],
            dtype=float,
        )
        S = H @ self.P @ H.T + R
        nis = float(nu @ np.linalg.solve(S, nu))

        if nis > self.p.bearing_gate_chi2:
            self.bearing_rejected += 1
            self._consec_bearing_rej += 1
            self._maybe_recover()
            return False
        self._consec_bearing_rej = 0

        self._apply(H, nu, S)
        self.bearing_accepted += 1
        self.last_bearing_stamp = meas.stamp
        return True

    def _maybe_recover(self) -> None:
        """Break an EKF death spiral by admitting the estimate may be wrong.

        Once the state has drifted far enough, every measurement falls outside
        the gate, nothing updates, and the covariance -- which only grows by
        process noise -- takes a long time to reopen the gate on its own. Worse,
        the freshness stamps only advance on ACCEPTED measurements, so the
        supervisor correctly reports the target as lost while the sensors are in
        fact working perfectly.

        Inflating the position/velocity covariance reopens the gate deliberately
        rather than waiting. It is the estimator equivalent of saying "I no
        longer trust where I think they are".
        """
        if max(self._consec_range_rej, self._consec_bearing_rej) < self.p.divergence_reject_limit:
            return
        self.P[:3, :3] *= self.p.divergence_inflation
        self.P[3:, 3:] *= self.p.divergence_inflation
        self._consec_range_rej = 0
        self._consec_bearing_rej = 0
        self.recoveries += 1

    def _apply(self, H: np.ndarray, nu: np.ndarray, S: np.ndarray) -> None:
        """Joseph-form covariance update -- keeps P symmetric positive definite."""
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        I_KH = np.eye(self.DIM) - K @ H
        R_eff = S - H @ self.P @ H.T
        self.P = I_KH @ self.P @ I_KH.T + K @ R_eff @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    # -- output -------------------------------------------------------------

    def estimate(self, health: TrackHealth = TrackHealth.TRACKING) -> TargetEstimate:
        return TargetEstimate(
            stamp=float(self.stamp if self.stamp is not None else 0.0),
            position=self.x[:3].copy(),
            velocity=self.x[3:].copy(),
            covariance=self.P.copy(),
            health=health,
        )
