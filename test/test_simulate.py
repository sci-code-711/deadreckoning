"""Tests for deadrec.simulate.

These verify the generator's own physics independently of
deadrec.dead_reckoning/deadrec.ekf/deadrec.kinematics/deadrec.attitude - by
hand-derived closed-form values and a standalone quaternion-to-rotation-matrix
helper (a different code path to deadrec.simulate's own quaternion
sandwich-product rotation) - so that using it later to validate those modules
isn't just testing them against themselves. See test_validate_algorithms.py
for the round-trip checks against the real reconstruction algorithms.

"""

import numpy as np
import pytest

from deadrec.io import read_imu_csv, write_imu_csv, write_trajectory_csv
from deadrec.quaternion import Quaternion
from deadrec.simulate import (
    STANDARD_GRAVITY,
    TRAJECTORIES,
    circular_turn,
    climbing_helix,
    constant_acceleration,
    constant_velocity,
    pure_rotation,
    sample_trajectory,
    stationary,
)


def _rotation_matrix(q):
    """
    An independent quaternion-to-rotation-matrix implementation (the
    standard closed-form DCM formula), used to cross-check
    deadrec.simulate's quaternion sandwich-product rotation via a different
    code path.

    """
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )


def test_stationary_reads_gravity_only():
    samples, truth = sample_trajectory(stationary(), duration_s=1.0, rate_hz=10)

    for sample in samples:
        assert np.allclose(sample.accel, [0.0, 0.0, STANDARD_GRAVITY])
        assert np.allclose(sample.gyro, [0.0, 0.0, 0.0])

    for state in truth:
        assert np.allclose(state.position, [0.0, 0.0, 0.0])
        assert np.allclose(state.velocity, [0.0, 0.0, 0.0])
        assert state.attitude == Quaternion(1.0, 0.0, 0.0, 0.0)


def test_constant_velocity_has_no_linear_acceleration():
    samples, truth = sample_trajectory(
        constant_velocity(velocity=(2.0, -1.0, 0.0)), duration_s=1.0, rate_hz=10
    )

    for sample in samples:
        assert np.allclose(sample.accel, [0.0, 0.0, STANDARD_GRAVITY])
        assert np.allclose(sample.gyro, [0.0, 0.0, 0.0])

    for state in truth:
        expected_position = np.array([2.0, -1.0, 0.0]) * state.t
        assert np.allclose(state.position, expected_position)
        assert np.allclose(state.velocity, [2.0, -1.0, 0.0])


def test_constant_acceleration_matches_hand_derivation():
    a = np.array([0.5, 0.0, 0.0])
    _, truth = sample_trajectory(constant_acceleration(acceleration=a), duration_s=2.0, rate_hz=10)

    for state in truth:
        t = state.t
        # p(t) = 1/2 a t^2, v(t) = a t - elementary kinematics, independent
        # of anything deadrec.simulate computes internally.
        assert np.allclose(state.position, 0.5 * a * t**2)
        assert np.allclose(state.velocity, a * t)
        assert np.allclose(state.accel_nav, a)


def test_pure_rotation_gyro_reading_matches_configured_rate():
    samples, truth = sample_trajectory(pure_rotation(rate_deg_s=30.0), duration_s=1.0, rate_hz=20)

    for sample in samples:
        assert np.allclose(sample.gyro, [0.0, 0.0, 30.0])

    for state in truth:
        # Closed form for a pure z-axis rotation by angle theta, computed
        # here independently of Quaternion.from_eul_angles.
        theta = np.radians(30.0) * state.t
        expected = (np.cos(theta / 2), 0.0, 0.0, np.sin(theta / 2))
        actual = (state.attitude.w, state.attitude.x, state.attitude.y, state.attitude.z)
        assert np.allclose(actual, expected)


def test_circular_turn_matches_independent_physics():
    speed, radius = 5.0, 20.0
    samples, truth = sample_trajectory(
        circular_turn(speed=speed, radius=radius), duration_s=2.0, rate_hz=50
    )

    expected_centripetal = speed**2 / radius

    for sample, state in zip(samples, truth):
        # Speed stays constant (uniform circular motion).
        assert np.linalg.norm(state.velocity) == pytest.approx(speed)
        # Centripetal acceleration magnitude v^2/R - elementary mechanics.
        assert np.linalg.norm(state.accel_nav) == pytest.approx(expected_centripetal)

        # Cross-check the body-frame accelerometer reading via the
        # independent rotation-matrix helper above, rather than
        # deadrec.simulate's own quaternion sandwich-product rotation.
        rotation = _rotation_matrix(state.attitude)
        expected_reading = rotation.T @ (state.accel_nav + [0.0, 0.0, STANDARD_GRAVITY])
        assert np.allclose(sample.accel, expected_reading, atol=1e-10)

    # Evaluate the closed-form position directly at t=period, rather than
    # via sample_trajectory's discretised time grid, which won't land on
    # an irrational period like 2*pi*R/v exactly.
    period = 2 * np.pi * radius / speed
    trajectory = circular_turn(speed=speed, radius=radius)
    assert np.allclose(trajectory.position(period), [0.0, 0.0, 0.0], atol=1e-8)


def test_climbing_helix_adds_constant_climb_to_circular_turn():
    climb_rate = 0.5
    _, helix_truth = sample_trajectory(
        climbing_helix(speed=5.0, radius=20.0, climb_rate=climb_rate), duration_s=2.0, rate_hz=50
    )
    _, turn_truth = sample_trajectory(
        circular_turn(speed=5.0, radius=20.0), duration_s=2.0, rate_hz=50
    )

    for helix_state, turn_state in zip(helix_truth, turn_truth):
        assert np.allclose(helix_state.position[:2], turn_state.position[:2])
        assert helix_state.position[2] == pytest.approx(climb_rate * helix_state.t)
        assert np.allclose(helix_state.accel_nav, turn_state.accel_nav)


def test_sample_trajectory_is_deterministic():
    samples_a, truth_a = sample_trajectory(
        circular_turn(),
        duration_s=1.0,
        rate_hz=20,
        noise_std_accel=0.05,
        noise_std_gyro=0.1,
        seed=42,
    )
    samples_b, truth_b = sample_trajectory(
        circular_turn(),
        duration_s=1.0,
        rate_hz=20,
        noise_std_accel=0.05,
        noise_std_gyro=0.1,
        seed=42,
    )

    for a, b in zip(samples_a, samples_b):
        assert a.t == b.t
        assert np.array_equal(a.accel, b.accel)
        assert np.array_equal(a.gyro, b.gyro)

    for a, b in zip(truth_a, truth_b):
        assert np.array_equal(a.position, b.position)


def test_noise_only_perturbs_readings_not_ground_truth():
    clean_samples, clean_truth = sample_trajectory(circular_turn(), duration_s=1.0, rate_hz=20)
    noisy_samples, noisy_truth = sample_trajectory(
        circular_turn(), duration_s=1.0, rate_hz=20, noise_std_accel=0.5, noise_std_gyro=1.0, seed=7
    )

    assert not np.allclose(
        [s.accel for s in noisy_samples], [s.accel for s in clean_samples], atol=1e-3
    )
    for clean_state, noisy_state in zip(clean_truth, noisy_truth):
        assert np.array_equal(clean_state.position, noisy_state.position)
        assert np.array_equal(clean_state.velocity, noisy_state.velocity)


@pytest.mark.parametrize("name", sorted(TRAJECTORIES))
def test_all_trajectories_produce_well_formed_output(name):
    samples, truth = sample_trajectory(TRAJECTORIES[name](), duration_s=1.0, rate_hz=25)

    assert len(samples) == len(truth) == 26  # duration * rate + 1

    times = np.array([s.t for s in samples])
    assert np.allclose(np.diff(times), 0.04)

    for sample in samples:
        assert np.all(np.isfinite(sample.accel))
        assert np.all(np.isfinite(sample.gyro))

    for state in truth:
        assert np.all(np.isfinite(state.position))
        assert np.all(np.isfinite(state.velocity))
        quat = state.attitude
        assert np.sqrt(quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2) == pytest.approx(1.0)


def test_sample_trajectory_rejects_invalid_duration_and_rate():
    with pytest.raises(ValueError):
        sample_trajectory(stationary(), duration_s=-1.0, rate_hz=10)

    with pytest.raises(ValueError):
        sample_trajectory(stationary(), duration_s=1.0, rate_hz=0)


def test_write_imu_csv_round_trips_through_read_imu_csv(tmp_path):
    samples, _ = sample_trajectory(circular_turn(), duration_s=1.0, rate_hz=20)
    path = tmp_path / "readings.csv"

    write_imu_csv(samples, path)
    round_tripped = read_imu_csv(path)

    assert len(round_tripped) == len(samples)
    for original, loaded in zip(samples, round_tripped):
        assert loaded.t == pytest.approx(original.t)
        assert np.allclose(loaded.accel, original.accel)
        assert np.allclose(loaded.gyro, original.gyro)


def test_write_trajectory_csv_has_one_row_per_state(tmp_path):
    _, truth = sample_trajectory(circular_turn(), duration_s=1.0, rate_hz=20)
    path = tmp_path / "truth.csv"

    write_trajectory_csv(truth, path)

    with open(path) as csv_file:
        lines = csv_file.readlines()

    assert len(lines) == len(truth) + 1  # header + one row per state
