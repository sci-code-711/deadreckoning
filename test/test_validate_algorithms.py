"""Round-trip validation: feed deadrec.simulate's synthetic trajectories
through the real reconstruction algorithms and check the results against
known ground truth.

deadrec.simulate is verified independently in test_simulate.py (hand-derived
values and a standalone rotation-matrix cross-check, with no dependency on
the algorithm modules below) - these tests then use it as a trusted source of
ground truth to validate deadrec.dead_reckoning/deadrec.attitude/deadrec.ekf
themselves.

"""

import numpy as np
import pytest

from deadrec.attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from deadrec.dead_reckoning import DeadReckoner
from deadrec.ekf import GravityCorrectedEKF, WindowedGravityCorrectedEKF
from deadrec.quaternion import Quaternion
from deadrec.simulate import (
    STANDARD_GRAVITY,
    circular_turn,
    climbing_helix,
    constant_acceleration,
    sample_trajectory,
    stationary,
)


def _position_errors(states, truth):
    return np.array(
        [np.linalg.norm(state.position - t.position) for state, t in zip(states, truth)]
    )


def _attitude_angle_error_deg(a: Quaternion, b: Quaternion) -> float:
    """Angle (degrees) between two attitude quaternions, accounting for the
    double cover (q and -q represent the same attitude)."""
    dot = abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z)
    return np.degrees(2 * np.arccos(np.clip(dot, -1.0, 1.0)))


def test_dead_reckoner_reconstructs_constant_acceleration_to_machine_precision():
    # Trapezoidal integration of a linearly-varying velocity is exact, so
    # with an exactly-known initial attitude/gravity magnitude the
    # reconstruction should match ground truth to near machine precision.
    samples, truth = sample_trajectory(
        constant_acceleration(acceleration=(0.5, -0.3, 0.2)), duration_s=5.0, rate_hz=50
    )

    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=STANDARD_GRAVITY)
    states = reckoner.run(samples)

    for state, expected in zip(states, truth):
        assert np.allclose(state.position, expected.position, atol=1e-8)
        assert np.allclose(state.velocity, expected.velocity, atol=1e-8)


def test_dead_reckoner_position_error_shrinks_with_finer_sampling():
    # Convergence check: RK4/trapezoidal integration error on a genuinely
    # curved (non-linear) trajectory should shrink as the sample rate
    # increases, rather than just happening to be "close enough".
    trajectory = circular_turn(speed=5.0, radius=20.0)

    coarse_samples, coarse_truth = sample_trajectory(trajectory, duration_s=4.0, rate_hz=20)
    fine_samples, fine_truth = sample_trajectory(trajectory, duration_s=4.0, rate_hz=200)

    # circular_turn doesn't start from rest, so seed the true initial
    # velocity/position - DeadReckoner's first step always seeds from
    # these (zero by default) rather than propagating, since there's no
    # previous sample to integrate against yet.
    coarse_states = DeadReckoner(
        Quaternion(1, 0, 0, 0),
        gravity_magnitude=STANDARD_GRAVITY,
        initial_velocity=coarse_truth[0].velocity,
        initial_position=coarse_truth[0].position,
    ).run(coarse_samples)
    fine_states = DeadReckoner(
        Quaternion(1, 0, 0, 0),
        gravity_magnitude=STANDARD_GRAVITY,
        initial_velocity=fine_truth[0].velocity,
        initial_position=fine_truth[0].position,
    ).run(fine_samples)

    coarse_error = _position_errors(coarse_states, coarse_truth).max()
    fine_error = _position_errors(fine_states, fine_truth).max()

    assert fine_error < coarse_error / 10


def test_dead_reckoner_tracks_climbing_helix_attitude_and_position():
    samples, truth = sample_trajectory(
        climbing_helix(speed=5.0, radius=20.0, climb_rate=0.5), duration_s=6.0, rate_hz=100
    )

    # climbing_helix doesn't start from rest either - see the note above.
    states = DeadReckoner(
        Quaternion(1, 0, 0, 0),
        gravity_magnitude=STANDARD_GRAVITY,
        initial_velocity=truth[0].velocity,
        initial_position=truth[0].position,
    ).run(samples)

    for state, expected in zip(states, truth):
        assert _attitude_angle_error_deg(state.attitude, expected.attitude) < 0.05
        assert np.linalg.norm(state.position - expected.position) < 0.01


def test_gravity_estimation_recovers_known_values_from_stationary_trajectory():
    samples, _ = sample_trajectory(stationary(), duration_s=2.0, rate_hz=100)
    accel = np.array([sample.accel for sample in samples])

    estimated_gravity = estimate_gravity_magnitude(accel)
    estimated_attitude = initial_attitude_from_gravity(accel[:30])

    assert estimated_gravity == pytest.approx(STANDARD_GRAVITY, abs=1e-9)
    assert _attitude_angle_error_deg(estimated_attitude, Quaternion(1, 0, 0, 0)) < 1e-6


def test_gravity_corrected_ekf_reduces_drift_from_noisy_gyro_vs_plain_dead_reckoning():
    # A stationary trajectory with noisy gyro readings: gyro-only
    # integration should drift away from the true (identity) attitude,
    # while the gravity-vector correction should keep the EKF variants
    # much closer to it - directly demonstrating the correction is
    # effective, not just present.
    samples, _ = sample_trajectory(
        stationary(), duration_s=30.0, rate_hz=50, noise_std_gyro=0.3, seed=1
    )

    plain_final = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=STANDARD_GRAVITY).run(
        samples
    )[-1]
    ekf_final = GravityCorrectedEKF(Quaternion(1, 0, 0, 0), gravity_magnitude=STANDARD_GRAVITY).run(
        samples
    )[-1]

    truth_attitude = Quaternion(1, 0, 0, 0)
    plain_error = _attitude_angle_error_deg(plain_final.attitude, truth_attitude)
    ekf_error = _attitude_angle_error_deg(ekf_final.attitude, truth_attitude)

    assert ekf_error < plain_error


def test_windowed_gravity_corrected_ekf_reduces_drift_from_noisy_gyro_vs_plain_dead_reckoning():
    samples, _ = sample_trajectory(
        stationary(), duration_s=30.0, rate_hz=50, noise_std_gyro=0.3, seed=1
    )

    plain_final = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=STANDARD_GRAVITY).run(
        samples
    )[-1]
    windowed_final = WindowedGravityCorrectedEKF(
        Quaternion(1, 0, 0, 0), gravity_magnitude=STANDARD_GRAVITY
    ).run(samples)[-1]

    truth_attitude = Quaternion(1, 0, 0, 0)
    plain_error = _attitude_angle_error_deg(plain_final.attitude, truth_attitude)
    windowed_error = _attitude_angle_error_deg(windowed_final.attitude, truth_attitude)

    assert windowed_error < plain_error
