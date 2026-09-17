import numpy as np
import pytest

from deadrec.benchmark_cases import (
    BENCHMARK_CASES,
    circular_motion,
    composite_sequence,
    constant_linear_acceleration,
    constant_rotation,
    full_composite,
    stationary,
)
from deadrec.quaternion import Quaternion
from deadrec.synthetic import SyntheticTrajectory


def test_registry_contains_all_six_cases():
    assert set(BENCHMARK_CASES) == {
        "stationary",
        "constant_rotation",
        "constant_linear_acceleration",
        "circular_motion",
        "composite_sequence",
        "full_composite",
    }


def test_every_registered_case_builds_a_synthetic_trajectory_with_positive_duration():
    for build in BENCHMARK_CASES.values():
        trajectory = build()
        assert isinstance(trajectory, SyntheticTrajectory)
        assert trajectory.duration > 0


def test_stationary_case():
    trajectory = stationary()
    assert trajectory.duration == pytest.approx(5.0)

    for t in [0.0, 2.5, 5.0]:
        state = trajectory.state_at(t)
        assert state.attitude == Quaternion(1, 0, 0, 0)
        assert np.allclose(state.velocity, [0, 0, 0])
        assert np.allclose(state.position, [0, 0, 0])
        assert np.allclose(state.accel_nav, [0, 0, 0])


def test_constant_rotation_case():
    trajectory = constant_rotation()
    assert trajectory.duration == pytest.approx(6.0)

    initial = trajectory.state_at(0.0)
    assert initial.attitude == Quaternion(1, 0, 0, 0)
    assert np.allclose(initial.position, [0, 0, 0])

    final = trajectory.state_at(6.0)
    expected_attitude = Quaternion.from_axis_angle([0, 0, 1], np.radians(30.0) * 6.0)
    assert final.attitude.w == pytest.approx(expected_attitude.w, abs=1e-9)
    assert final.attitude.z == pytest.approx(expected_attitude.z, abs=1e-9)
    # No translation at any point - pure rotation in place.
    assert np.allclose(final.position, [0, 0, 0])


def test_constant_linear_acceleration_case():
    trajectory = constant_linear_acceleration()
    assert trajectory.duration == pytest.approx(5.0)

    initial = trajectory.state_at(0.0)
    assert initial.attitude == Quaternion(1, 0, 0, 0)
    assert np.allclose(initial.velocity, [0, 0, 0])
    assert np.allclose(initial.position, [0, 0, 0])

    final = trajectory.state_at(5.0)
    assert np.allclose(final.velocity, [10.0, 0.0, 0.0])  # a=2 * t=5
    assert np.allclose(final.position, [25.0, 0.0, 0.0])  # 0.5 * a * t^2
    assert final.attitude == Quaternion(1, 0, 0, 0)  # attitude fixed throughout


def test_circular_motion_case():
    trajectory = circular_motion()
    assert trajectory.duration == pytest.approx(9.0)

    initial = trajectory.state_at(0.0)
    assert np.allclose(initial.velocity, [5.0, 0.0, 0.0])

    # 9s at 20 deg/s = 180 degrees - a full half-circle, ending diametrically
    # opposite the start with velocity exactly reversed.
    final = trajectory.state_at(9.0)
    radius = 5.0 / np.radians(20.0)
    assert np.allclose(final.velocity, [-5.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(final.position, [0.0, 2 * radius, 0.0], atol=1e-9)


def test_composite_sequence_case_ends_at_rest():
    trajectory = composite_sequence()
    assert trajectory.duration == pytest.approx(16.0)

    initial = trajectory.state_at(0.0)
    assert np.allclose(initial.velocity, [0, 0, 0])
    assert np.allclose(initial.position, [0, 0, 0])

    # accelerate phase: 0 -> 6 m/s over 3s at 2 m/s^2
    after_accelerate = trajectory.state_at(3.0)
    assert np.allclose(after_accelerate.velocity, [6.0, 0.0, 0.0])

    # cruise phase holds speed
    after_cruise = trajectory.state_at(7.0)
    assert np.allclose(after_cruise.velocity, [6.0, 0.0, 0.0])

    # decelerate phase: 6 -> 3 m/s over 3s
    after_decelerate = trajectory.state_at(10.0)
    assert np.allclose(after_decelerate.velocity, [3.0, 0.0, 0.0])

    # turn phase: 90 degrees at 30 deg/s over 3s, constant 3 m/s speed
    after_turn = trajectory.state_at(13.0)
    assert np.allclose(after_turn.velocity, [0.0, 3.0, 0.0], atol=1e-9)

    # final stop phase: 3 -> 0 m/s over 3s, known final state.
    final = trajectory.state_at(16.0)
    assert np.allclose(final.velocity, [0.0, 0.0, 0.0], atol=1e-9)


def test_full_composite_case_moves_and_rotates_off_axis():
    trajectory = full_composite()
    assert trajectory.duration == pytest.approx(8.0)

    final = trajectory.state_at(8.0)
    # A stress-test sanity check, not an exact value: multi-axis rotation
    # and translation should leave every position/attitude axis disturbed.
    assert np.all(np.abs(final.position) > 1e-6)
    assert abs(final.attitude.x) > 1e-6
    assert abs(final.attitude.y) > 1e-6
    assert abs(final.attitude.z) > 1e-6
