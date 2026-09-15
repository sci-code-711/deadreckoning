import math

import numpy as np
import pytest

from deadrec.kinematics import omega_matrix, rk4_attitude_step
from deadrec.quaternion import Quaternion


def test_omega_matrix_values():
    expected = np.array(
        [
            [0, -1, -2, -3],
            [1, 0, 3, -2],
            [2, -3, 0, 1],
            [3, 2, -1, 0],
        ]
    )

    assert np.array_equal(omega_matrix([1.0, 2.0, 3.0]), expected)


def test_omega_matrix_is_skew_symmetric():
    mat = omega_matrix([1.0, -2.0, 0.5])

    assert np.allclose(mat, -mat.T)


def test_omega_matrix_rejects_wrong_shape():
    with pytest.raises(ValueError):
        omega_matrix([1.0, 2.0])


def test_rk4_attitude_step_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = rk4_attitude_step(qi, [0, 0, 0], [0, 0, 0], 0.05)

    assert result.w == pytest.approx(qi.w)
    assert result.x == pytest.approx(qi.x)
    assert result.y == pytest.approx(qi.y)
    assert result.z == pytest.approx(qi.z)


def test_rk4_attitude_step_returns_unit_quaternion():
    result = rk4_attitude_step(Quaternion(1, 0, 0, 0), [10, 20, 30], [15, 25, 20], 0.02)

    assert abs(result) == pytest.approx(1.0)


def test_rk4_attitude_step_matches_closed_form_single_axis_rotation():
    # A constant angular rate about a single axis has a closed-form solution:
    # rotation by angle |w| * dt about that axis. For small dt the RK4
    # truncation error should be negligible.
    wz_deg = 90.0
    dt = 0.01
    wz_rad = math.radians(wz_deg)

    result = rk4_attitude_step(Quaternion(1, 0, 0, 0), [0, 0, wz_deg], [0, 0, wz_deg], dt)
    expected = Quaternion.from_eul_angles(0, 0, wz_rad * dt / 2)

    assert result.w == pytest.approx(expected.w, abs=1e-9)
    assert result.x == pytest.approx(expected.x, abs=1e-9)
    assert result.y == pytest.approx(expected.y, abs=1e-9)
    assert result.z == pytest.approx(expected.z, abs=1e-9)


def test_rk4_attitude_step_composes_with_existing_attitude():
    # Rotating twice by dt/2 with a constant rate should match rotating once
    # by dt (up to RK4 truncation error).
    wz_deg = 45.0
    dt = 0.02

    once = rk4_attitude_step(Quaternion(1, 0, 0, 0), [0, 0, wz_deg], [0, 0, wz_deg], dt)
    twice = rk4_attitude_step(Quaternion(1, 0, 0, 0), [0, 0, wz_deg], [0, 0, wz_deg], dt / 2)
    twice = rk4_attitude_step(twice, [0, 0, wz_deg], [0, 0, wz_deg], dt / 2)

    assert once.w == pytest.approx(twice.w, abs=1e-8)
    assert once.x == pytest.approx(twice.x, abs=1e-8)
    assert once.y == pytest.approx(twice.y, abs=1e-8)
    assert once.z == pytest.approx(twice.z, abs=1e-8)
