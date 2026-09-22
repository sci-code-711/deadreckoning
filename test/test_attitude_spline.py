import numpy as np
import pytest

from deadrec.attitude_spline import AttitudeSpline
from deadrec.quaternion import Quaternion
from deadrec.synthetic import MotionSegment, SyntheticTrajectory

_G = 9.80665


def _assert_quaternions_close(a, b, abs_tol=1e-9):
    assert a.w == pytest.approx(b.w, abs=abs_tol)
    assert a.x == pytest.approx(b.x, abs=abs_tol)
    assert a.y == pytest.approx(b.y, abs=abs_tol)
    assert a.z == pytest.approx(b.z, abs=abs_tol)


def test_attitude_spline_requires_at_least_two_keyframes():
    with pytest.raises(ValueError):
        AttitudeSpline([(0.0, Quaternion(1, 0, 0, 0))])


def test_attitude_spline_requires_strictly_increasing_times():
    with pytest.raises(ValueError):
        AttitudeSpline(
            [
                (0.0, Quaternion(1, 0, 0, 0)),
                (0.0, Quaternion.from_axis_angle([0, 0, 1], 0.1)),
            ]
        )

    with pytest.raises(ValueError):
        AttitudeSpline(
            [
                (1.0, Quaternion(1, 0, 0, 0)),
                (0.0, Quaternion.from_axis_angle([0, 0, 1], 0.1)),
            ]
        )


def test_attitude_spline_passes_through_every_keyframe_exactly():
    keyframes = [
        (0.0, Quaternion(1, 0, 0, 0)),
        (1.0, Quaternion.from_axis_angle([0, 0, 1], np.pi / 4)),
        (2.0, Quaternion.from_axis_angle([1, 0, 0], np.pi / 6)),
        (3.0, Quaternion.from_axis_angle([0, 1, 0], np.pi / 3)),
    ]
    spline = AttitudeSpline(keyframes)

    for t, q in keyframes:
        _assert_quaternions_close(spline.attitude(t), q)


def test_attitude_spline_rejects_time_outside_its_range():
    spline = AttitudeSpline(
        [(0.0, Quaternion(1, 0, 0, 0)), (1.0, Quaternion.from_axis_angle([0, 0, 1], 0.5))]
    )

    with pytest.raises(ValueError):
        spline.attitude(-0.1)
    with pytest.raises(ValueError):
        spline.attitude(1.1)


def test_attitude_spline_matches_constant_rotation_closed_form():
    wz_deg = 30.0
    wz_rad = np.radians(wz_deg)
    keyframe_times = [0.0, 1.0, 2.0, 3.0, 4.0]
    keyframes = [(t, Quaternion.from_axis_angle([0, 0, 1], wz_rad * t)) for t in keyframe_times]
    spline = AttitudeSpline(keyframes)

    reference = SyntheticTrajectory(
        [MotionSegment(duration=4.0, angular_velocity_body=[0, 0, wz_deg])],
        gravity_magnitude=_G,
    )

    for t in [0.3, 1.7, 2.5, 3.9]:
        expected_attitude = reference.state_at(t).attitude
        _assert_quaternions_close(spline.attitude(t), expected_attitude, abs_tol=1e-8)

        expected_rate = reference.imu_sample_at(t).gyro
        actual_rate = spline.angular_velocity_body(t)
        assert np.allclose(actual_rate, expected_rate, atol=1e-4)


def test_attitude_spline_angular_velocity_matches_independent_finite_difference():
    keyframes = [
        (0.0, Quaternion(1, 0, 0, 0)),
        (1.0, Quaternion.from_axis_angle([0, 0, 1], np.pi / 3)),
        (2.0, Quaternion.from_axis_angle([1, 1, 0], np.pi / 4)),
        (3.0, Quaternion.from_axis_angle([0, 1, 1], np.pi / 5)),
    ]
    spline = AttitudeSpline(keyframes)

    t = 1.4
    h = 1e-5  # deliberately different from the class's own fixed step

    q_minus = spline.attitude(t - h)
    q_plus = spline.attitude(t + h)

    delta = q_minus.conjugate() * q_plus
    delta = delta * (1.0 / abs(delta))
    v = np.array([delta.x, delta.y, delta.z])
    angle = 2 * np.arctan2(np.linalg.norm(v), delta.w)
    axis = v / np.linalg.norm(v)
    expected_rate = np.degrees(axis * angle / (2 * h))

    actual_rate = spline.angular_velocity_body(t)

    assert np.allclose(actual_rate, expected_rate, atol=1e-2)


def test_attitude_spline_angular_velocity_is_zero_for_a_stationary_curve():
    q = Quaternion.from_axis_angle([1, 0, 0], 0.4)
    spline = AttitudeSpline([(0.0, q), (1.0, q), (2.0, q)])

    assert np.allclose(spline.angular_velocity_body(1.0), [0.0, 0.0, 0.0], atol=1e-6)
