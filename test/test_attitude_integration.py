import math

import numpy as np
import pytest

from deadrec.attitude_integration import RK4Integrator
from deadrec.interpolation import TwoPointLinearInterpolator
from deadrec.kinematics import rk4_attitude_step
from deadrec.quaternion import Quaternion
from deadrec.samples import ImuSample


def _integrate(q0, w1, w2, dt, t0=0.0):
    """RK4Integrator + TwoPointLinearInterpolator, driven the same way
    rk4_attitude_step(q0, w1, w2, dt) is - w1/w2 in degrees/s."""
    window = [
        ImuSample(t=t0, accel=[0.0, 0.0, 0.0], gyro=w1),
        ImuSample(t=t0 + dt, accel=[0.0, 0.0, 0.0], gyro=w2),
    ]
    omega = TwoPointLinearInterpolator().build(window, step_pos=1)
    return RK4Integrator().integrate(q0, omega, window[0].t, window[1].t)


def _assert_quaternions_close(a, b, abs_tol=1e-9):
    assert a.w == pytest.approx(b.w, abs=abs_tol)
    assert a.x == pytest.approx(b.x, abs=abs_tol)
    assert a.y == pytest.approx(b.y, abs=abs_tol)
    assert a.z == pytest.approx(b.z, abs=abs_tol)


def test_rk4_integrator_matches_rk4_attitude_step_zero_rate():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    expected = rk4_attitude_step(qi, [0, 0, 0], [0, 0, 0], 0.05)
    actual = _integrate(qi, [0, 0, 0], [0, 0, 0], 0.05)

    _assert_quaternions_close(actual, expected)


def test_rk4_integrator_matches_rk4_attitude_step_returns_unit_quaternion():
    expected = rk4_attitude_step(Quaternion(1, 0, 0, 0), [10, 20, 30], [15, 25, 20], 0.02)
    actual = _integrate(Quaternion(1, 0, 0, 0), [10, 20, 30], [15, 25, 20], 0.02)

    assert abs(actual) == pytest.approx(1.0)
    _assert_quaternions_close(actual, expected)


def test_rk4_integrator_matches_closed_form_single_axis_rotation():
    wz_deg = 90.0
    dt = 0.01
    wz_rad = math.radians(wz_deg)

    actual = _integrate(Quaternion(1, 0, 0, 0), [0, 0, wz_deg], [0, 0, wz_deg], dt)
    expected = Quaternion.from_eul_angles(0, 0, wz_rad * dt / 2)

    _assert_quaternions_close(actual, expected)


def test_rk4_integrator_matches_rk4_attitude_step_composability():
    wz_deg = 45.0
    dt = 0.02

    expected_once = rk4_attitude_step(Quaternion(1, 0, 0, 0), [0, 0, wz_deg], [0, 0, wz_deg], dt)

    twice = _integrate(Quaternion(1, 0, 0, 0), [0, 0, wz_deg], [0, 0, wz_deg], dt / 2)
    twice = _integrate(twice, [0, 0, wz_deg], [0, 0, wz_deg], dt / 2, t0=dt / 2)

    _assert_quaternions_close(twice, expected_once, abs_tol=1e-8)


@pytest.mark.parametrize(
    "w1,w2,dt",
    [
        ([0, 0, 0], [0, 0, 0], 0.05),
        ([10, 20, 30], [15, 25, 20], 0.02),
        ([0, 0, 90.0], [0, 0, 90.0], 0.01),
        ([5, -3, 12], [-8, 4, 6], 0.1),
    ],
)
def test_rk4_integrator_matches_rk4_attitude_step_across_cases(w1, w2, dt):
    qi = Quaternion.from_eul_angles(0.1, 0.2, -0.3)

    expected = rk4_attitude_step(qi, w1, w2, dt)
    actual = _integrate(qi, w1, w2, dt)

    _assert_quaternions_close(actual, expected)


def test_rk4_integrator_queries_omega_at_start_midpoint_and_end():
    calls = []

    def omega(t):
        calls.append(t)
        return np.zeros(3)

    RK4Integrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.1)

    assert calls == [0.0, 0.05, 0.05, 0.1]
