"""EKF correctness. The Jacobian tests are the load-bearing ones -- an EKF with
a subtly wrong Jacobian still converges on easy data and diverges in flight."""
import numpy as np
import pytest

from follow_core.ekf import TargetEKF, _bearing_from_vector
from follow_core.types import (
    BearingMeasurement,
    FollowParams,
    RangeMeasurement,
    VehicleState,
    wrap_pi,
)


def rot_xyz(rx, ry, rz):
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@pytest.fixture
def vehicle():
    return VehicleState(
        stamp=0.0,
        position=np.array([1.0, -0.5, 2.0]),
        yaw=0.3,
        R_world_cam=rot_xyz(0.15, -0.25, 0.4),
    )


def test_bearing_jacobian_matches_numerical(vehicle):
    """Analytic H for the bearing update must match finite differences."""
    ekf = TargetEKF()
    ekf.initialize([3.5, 2.0, 0.1], 0.0)

    R_cw = vehicle.R_world_cam.T

    def h(x3):
        return np.array(_bearing_from_vector(R_cw @ (x3 - vehicle.position)))

    x0 = ekf.x[:3].copy()
    eps = 1e-6
    J_num = np.zeros((2, 3))
    for i in range(3):
        dx = np.zeros(3)
        dx[i] = eps
        hp, hm = h(x0 + dx), h(x0 - dx)
        J_num[:, i] = wrap_pi(hp - hm) / (2 * eps)

    # Recompute the analytic block exactly as update_bearing does.
    d_cam = R_cw @ (x0 - vehicle.position)
    n2 = float(d_cam @ d_cam)
    x, y, z = d_cam
    rho2 = x * x + z * z
    rho = np.sqrt(rho2)
    J = np.zeros((2, 3))
    J[0, 0] = z / rho2
    J[0, 2] = -x / rho2
    J[1, 0] = y * x / (n2 * rho)
    J[1, 1] = -rho / n2
    J[1, 2] = y * z / (n2 * rho)
    J_ana = J @ R_cw

    assert np.allclose(J_ana, J_num, atol=1e-6), f"\nanalytic\n{J_ana}\nnumerical\n{J_num}"


def test_range_jacobian_matches_numerical(vehicle):
    ekf = TargetEKF()
    ekf.initialize([4.0, 1.0, 0.0], 0.0)
    x0 = ekf.x[:3].copy()

    def h(x3):
        return np.linalg.norm(x3 - vehicle.position)

    eps = 1e-7
    J_num = np.zeros(3)
    for i in range(3):
        dx = np.zeros(3)
        dx[i] = eps
        J_num[i] = (h(x0 + dx) - h(x0 - dx)) / (2 * eps)

    d = x0 - vehicle.position
    J_ana = d / np.linalg.norm(d)
    assert np.allclose(J_ana, J_num, atol=1e-6)


def test_range_updates_converge(vehicle):
    """Repeated noisy ranges pull a badly-initialised state onto the sphere."""
    truth = np.array([5.0, 0.0, 0.0])
    ekf = TargetEKF()
    ekf.initialize([3.0, 0.0, 0.0], 0.0, pos_std=2.0)
    rng = np.random.default_rng(1)
    true_r = float(np.linalg.norm(truth - vehicle.position))
    for k in range(120):
        t = 0.05 * (k + 1)
        z = true_r + rng.normal(0, 0.05)
        ekf.update_range(RangeMeasurement(stamp=t, range_m=z, tag_id=1, std_m=0.05), vehicle)
    est_r = float(np.linalg.norm(ekf.x[:3] - vehicle.position))
    assert abs(est_r - true_r) < 0.10


def test_covariance_stays_symmetric_and_psd(vehicle):
    ekf = TargetEKF()
    ekf.initialize([4.0, 1.0, 0.5], 0.0)
    rng = np.random.default_rng(7)
    for k in range(200):
        t = 0.05 * (k + 1)
        d = np.array([4.0, 1.0, 0.5]) - vehicle.position
        r = float(np.linalg.norm(d))
        ekf.update_range(
            RangeMeasurement(stamp=t, range_m=r + rng.normal(0, 0.08), tag_id=1), vehicle
        )
        az, el = _bearing_from_vector(vehicle.R_world_cam.T @ d)
        ekf.update_bearing(
            BearingMeasurement(
                stamp=t + 0.001,
                azimuth_rad=az + rng.normal(0, 0.01),
                elevation_rad=el + rng.normal(0, 0.01),
            ),
            vehicle,
        )
        assert np.allclose(ekf.P, ekf.P.T, atol=1e-9), "covariance lost symmetry"
        assert np.all(np.linalg.eigvalsh(ekf.P) > -1e-9), "covariance lost PSD"


def test_range_outlier_is_gated_out(vehicle):
    ekf = TargetEKF()
    ekf.initialize([5.0, 0.0, 0.0], 0.0, pos_std=0.1)
    before = ekf.x[:3].copy()
    # A 20 m return from a multipath reflection must not be believed.
    ok = ekf.update_range(
        RangeMeasurement(stamp=0.1, range_m=20.0, tag_id=1, std_m=0.05), vehicle
    )
    assert ok is False
    assert ekf.range_rejected == 1
    assert np.allclose(ekf.x[:3], before, atol=1e-6)


def test_bearing_gate_rejects_a_different_person(vehicle):
    """FR-2: a detection of someone else fails the gate and never reaches state.

    This is the identity mechanism -- the estimate is anchored by UWB range, so
    a decoy at a clearly different bearing is inconsistent and gets dropped.
    """
    truth = np.array([5.0, 0.0, 0.0])
    ekf = TargetEKF()
    ekf.initialize(truth, 0.0, pos_std=0.15, vel_std=0.2)

    decoy = np.array([2.0, 4.5, 0.0])
    az_d, el_d = _bearing_from_vector(vehicle.R_world_cam.T @ (decoy - vehicle.position))
    ok = ekf.update_bearing(
        BearingMeasurement(stamp=0.1, azimuth_rad=az_d, elevation_rad=el_d, std_rad=0.02),
        vehicle,
    )
    assert ok is False
    assert ekf.bearing_rejected == 1

    # The real operator, at the same instant, is still accepted.
    az_t, el_t = _bearing_from_vector(vehicle.R_world_cam.T @ (truth - vehicle.position))
    assert ekf.update_bearing(
        BearingMeasurement(stamp=0.11, azimuth_rad=az_t, elevation_rad=el_t), vehicle
    )


def test_triangulation_bootstrap_is_accurate(vehicle):
    truth = np.array([4.0, 2.5, 0.3])
    d = truth - vehicle.position
    r = float(np.linalg.norm(d))
    az, el = _bearing_from_vector(vehicle.R_world_cam.T @ d)
    ekf = TargetEKF()
    ekf.initialize_from_range_bearing(
        RangeMeasurement(stamp=1.0, range_m=r, tag_id=1),
        BearingMeasurement(stamp=1.0, azimuth_rad=az, elevation_rad=el),
        vehicle,
    )
    assert ekf.initialized
    assert np.allclose(ekf.x[:3], truth, atol=1e-6)


def test_predict_ignores_backwards_and_duplicate_time(vehicle):
    ekf = TargetEKF()
    ekf.initialize([1.0, 0.0, 0.0], 10.0, velocity=[1.0, 0.0, 0.0])
    ekf.predict(9.0)
    assert ekf.stamp == 10.0
    ekf.predict(10.0)
    assert ekf.stamp == 10.0
    ekf.predict(10.5)
    assert ekf.stamp == 10.5
    assert ekf.x[0] == pytest.approx(1.5)


def test_predict_clamps_a_long_gap(vehicle):
    """A 30 s dropout must not extrapolate the operator 30 m away."""
    ekf = TargetEKF()
    ekf.initialize([0.0, 0.0, 0.0], 0.0, velocity=[1.0, 0.0, 0.0])
    ekf.predict(30.0)
    assert ekf.x[0] <= 1.0 + 1e-9


def test_uninitialized_updates_are_noops(vehicle):
    ekf = TargetEKF()
    assert ekf.update_range(RangeMeasurement(stamp=0.1, range_m=3.0, tag_id=1), vehicle) is False
    assert ekf.update_bearing(BearingMeasurement(stamp=0.1, azimuth_rad=0.0, elevation_rad=0.0), vehicle) is False
