import numpy as np
import pytest

from deadrec.dead_reckoning import accel_to_nav_frame
from deadrec.quaternion import Quaternion
from deadrec.synthetic import CoordinatedTurnSegment, MotionSegment, SyntheticTrajectory

_G = 9.80665


# --- Independent reference implementation, used only to cross-check the
# closed-form segment math: a plain fixed-step numerical integrator that
# does *not* reuse SyntheticTrajectory's own math (or DeadReckoner's RK4),
# so a bug shared between the two wouldn't cancel out.


def _numerically_integrate(segments, initial_attitude, initial_velocity, initial_position, dt=1e-5):
    attitude = initial_attitude
    velocity = np.asarray(initial_velocity, dtype=float)
    position = np.asarray(initial_position, dtype=float)

    for segment in segments:
        n_steps = int(round(segment.duration / dt))
        rate_rad = np.radians(segment.angular_velocity_body)

        for _ in range(n_steps):
            half_angle = np.linalg.norm(rate_rad) * dt / 2
            if np.linalg.norm(rate_rad) > 0:
                axis = rate_rad / np.linalg.norm(rate_rad)
                delta = Quaternion(np.cos(half_angle), *(axis * np.sin(half_angle)))
                attitude = attitude * delta
                attitude = attitude * (1.0 / abs(attitude))

            position = position + velocity * dt + 0.5 * segment.accel_nav * dt**2
            velocity = velocity + segment.accel_nav * dt

    return attitude, velocity, position


def test_motion_segment_rejects_non_positive_duration():
    with pytest.raises(ValueError):
        MotionSegment(duration=0)
    with pytest.raises(ValueError):
        MotionSegment(duration=-1)


def test_motion_segment_rejects_wrong_shape_vectors():
    with pytest.raises(ValueError):
        MotionSegment(duration=1, accel_nav=[1, 2])
    with pytest.raises(ValueError):
        MotionSegment(duration=1, angular_velocity_body=[1, 2, 3, 4])


def test_trajectory_rejects_empty_segments():
    with pytest.raises(ValueError):
        SyntheticTrajectory([])


def test_stationary_trajectory_has_zero_drift():
    trajectory = SyntheticTrajectory([MotionSegment(duration=5.0)], gravity_magnitude=_G)

    for t in [0.0, 1.0, 2.5, 5.0]:
        state = trajectory.state_at(t)
        assert state.attitude == Quaternion(1, 0, 0, 0)
        assert np.allclose(state.velocity, [0, 0, 0])
        assert np.allclose(state.position, [0, 0, 0])
        assert np.allclose(state.accel_nav, [0, 0, 0])

        sample = trajectory.imu_sample_at(t)
        assert np.allclose(sample.accel, [0, 0, _G])
        assert np.allclose(sample.gyro, [0, 0, 0])


def test_constant_rotation_matches_closed_form():
    wz_deg = 30.0
    trajectory = SyntheticTrajectory(
        [MotionSegment(duration=3.0, angular_velocity_body=[0, 0, wz_deg])],
        gravity_magnitude=_G,
    )

    state = trajectory.state_at(2.0)
    expected = Quaternion.from_axis_angle([0, 0, 1], np.radians(wz_deg) * 2.0)

    assert state.attitude.w == pytest.approx(expected.w, abs=1e-12)
    assert state.attitude.x == pytest.approx(expected.x, abs=1e-12)
    assert state.attitude.y == pytest.approx(expected.y, abs=1e-12)
    assert state.attitude.z == pytest.approx(expected.z, abs=1e-12)
    assert np.allclose(state.position, [0, 0, 0])

    sample = trajectory.imu_sample_at(2.0)
    assert np.allclose(sample.gyro, [0, 0, wz_deg])


def test_constant_acceleration_matches_closed_form():
    accel = np.array([1.0, -0.5, 0.0])
    trajectory = SyntheticTrajectory(
        [MotionSegment(duration=4.0, accel_nav=accel)], gravity_magnitude=_G
    )

    state = trajectory.state_at(2.0)
    assert np.allclose(state.velocity, accel * 2.0)
    assert np.allclose(state.position, 0.5 * accel * 2.0**2)
    assert np.allclose(state.accel_nav, accel)


def test_imu_sample_round_trips_through_accel_to_nav_frame():
    # The body-frame reading synthesized by imu_sample_at should, when run
    # back through the existing (already-tested) accel_to_nav_frame, exactly
    # recover the segment's true nav-frame acceleration - this is the crux
    # correctness property for the reading-synthesis inverse.
    trajectory = SyntheticTrajectory(
        [
            MotionSegment(
                duration=2.0, accel_nav=[0.5, 1.2, -0.3], angular_velocity_body=[10, -5, 20]
            ),
        ],
        initial_attitude=Quaternion.from_eul_angles(0.2, -0.3, 0.4),
        gravity_magnitude=_G,
    )

    for t in [0.0, 0.7, 1.3, 2.0]:
        state = trajectory.state_at(t)
        sample = trajectory.imu_sample_at(t)
        recovered = accel_to_nav_frame(sample.accel, state.attitude, _G)
        assert np.allclose(recovered, state.accel_nav, atol=1e-9)


def test_multi_segment_chain_matches_independent_numerical_integration():
    segments = [
        MotionSegment(duration=1.0, accel_nav=[1.0, 0.0, 0.0], angular_velocity_body=[0, 0, 45]),
        MotionSegment(duration=0.5, accel_nav=[0.0, -2.0, 0.5], angular_velocity_body=[10, 0, -20]),
        MotionSegment(duration=1.5, accel_nav=[0.0, 0.0, 0.0], angular_velocity_body=[0, 0, 0]),
    ]
    initial_attitude = Quaternion.from_eul_angles(0.1, 0.05, -0.2)
    initial_velocity = [0.5, -0.2, 0.1]
    initial_position = [1.0, 2.0, -1.0]

    trajectory = SyntheticTrajectory(
        segments,
        initial_attitude=initial_attitude,
        initial_velocity=initial_velocity,
        initial_position=initial_position,
        gravity_magnitude=_G,
    )

    ref_attitude, ref_velocity, ref_position = _numerically_integrate(
        segments, initial_attitude, initial_velocity, initial_position
    )

    final_state = trajectory.state_at(trajectory.duration)
    assert final_state.attitude.w == pytest.approx(ref_attitude.w, abs=1e-6)
    assert final_state.attitude.x == pytest.approx(ref_attitude.x, abs=1e-6)
    assert final_state.attitude.y == pytest.approx(ref_attitude.y, abs=1e-6)
    assert final_state.attitude.z == pytest.approx(ref_attitude.z, abs=1e-6)
    assert np.allclose(final_state.velocity, ref_velocity, atol=1e-6)
    assert np.allclose(final_state.position, ref_position, atol=1e-6)


def test_sample_evaluates_all_times():
    trajectory = SyntheticTrajectory(
        [
            MotionSegment(duration=1.0, accel_nav=[1.0, 0.0, 0.0]),
            MotionSegment(duration=1.0, angular_velocity_body=[0, 0, 10]),
        ],
        gravity_magnitude=_G,
    )

    ground_truth, samples = trajectory.sample([0.0, 0.5, 1.0, 1.5, 2.0])

    assert len(ground_truth) == 5
    assert len(samples) == 5
    assert [state.t for state in ground_truth] == [0.0, 0.5, 1.0, 1.5, 2.0]
    assert [sample.t for sample in samples] == [0.0, 0.5, 1.0, 1.5, 2.0]


def test_sample_at_rate_covers_full_duration():
    trajectory = SyntheticTrajectory(
        [MotionSegment(duration=2.0, accel_nav=[1.0, 0.0, 0.0])], gravity_magnitude=_G
    )

    ground_truth, samples = trajectory.sample_at_rate(hz=10)

    assert len(ground_truth) == 21  # 0.0, 0.1, ..., 2.0 inclusive
    assert ground_truth[0].t == pytest.approx(0.0)
    assert ground_truth[-1].t == pytest.approx(2.0)
    assert samples[-1].t == pytest.approx(2.0)


def test_sample_at_different_rates_agree_on_ground_truth():
    # Resampling at a different rate must not change the ground truth at
    # shared time points - the whole point of closed-form generation.
    trajectory = SyntheticTrajectory(
        [
            MotionSegment(
                duration=1.0, accel_nav=[0.3, -0.1, 0.2], angular_velocity_body=[5, 10, -15]
            ),
        ],
        gravity_magnitude=_G,
    )

    slow_truth, _ = trajectory.sample_at_rate(hz=10)
    fast_truth, _ = trajectory.sample_at_rate(hz=100)

    slow_at_half = next(s for s in slow_truth if s.t == pytest.approx(0.5))
    fast_at_half = next(s for s in fast_truth if s.t == pytest.approx(0.5))

    assert np.allclose(slow_at_half.position, fast_at_half.position)
    assert np.allclose(slow_at_half.velocity, fast_at_half.velocity)


def test_evaluate_rejects_times_outside_duration():
    trajectory = SyntheticTrajectory([MotionSegment(duration=1.0)], gravity_magnitude=_G)

    with pytest.raises(ValueError):
        trajectory.state_at(-0.1)
    with pytest.raises(ValueError):
        trajectory.state_at(1.1)


def test_sample_at_rate_rejects_non_positive_hz():
    trajectory = SyntheticTrajectory([MotionSegment(duration=1.0)], gravity_magnitude=_G)

    with pytest.raises(ValueError):
        trajectory.sample_at_rate(0)
    with pytest.raises(ValueError):
        trajectory.sample_at_rate(-5)


# --- CoordinatedTurnSegment (circular motion) ---


def _numerically_integrate_turn(duration, turn_rate_deg, velocity0, position0, dt=1e-5):
    # Independent cross-check: dv/dt = omega x v for rotation about z,
    # rather than the closed-form trig solution CoordinatedTurnSegment uses.
    omega = np.radians(turn_rate_deg)
    velocity = np.asarray(velocity0, dtype=float)
    position = np.asarray(position0, dtype=float)
    n_steps = int(round(duration / dt))

    for _ in range(n_steps):
        vx, vy, vz = velocity
        accel = np.array([-vy * omega, vx * omega, 0.0])
        position = position + velocity * dt + 0.5 * accel * dt**2
        velocity = velocity + accel * dt

    return velocity, position


def test_coordinated_turn_rejects_non_positive_duration():
    with pytest.raises(ValueError):
        CoordinatedTurnSegment(duration=0, turn_rate_deg=10)
    with pytest.raises(ValueError):
        CoordinatedTurnSegment(duration=-1, turn_rate_deg=10)


def test_coordinated_turn_matches_independent_numerical_integration():
    initial_velocity = [5.0, 0.0, 0.0]
    turn_rate_deg = 20.0
    duration = 3.0

    trajectory = SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=duration, turn_rate_deg=turn_rate_deg)],
        initial_velocity=initial_velocity,
        gravity_magnitude=_G,
    )

    final_state = trajectory.state_at(duration)
    ref_velocity, ref_position = _numerically_integrate_turn(
        duration, turn_rate_deg, initial_velocity, [0.0, 0.0, 0.0]
    )

    assert np.allclose(final_state.velocity, ref_velocity, atol=1e-6)
    assert np.allclose(final_state.position, ref_position, atol=1e-6)


def test_coordinated_turn_holds_constant_speed():
    trajectory = SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=5.0, turn_rate_deg=15.0)],
        initial_velocity=[3.0, 4.0, 0.0],
        gravity_magnitude=_G,
    )
    speed0 = np.hypot(3.0, 4.0)

    for t in [0.0, 1.0, 2.5, 5.0]:
        state = trajectory.state_at(t)
        assert np.hypot(state.velocity[0], state.velocity[1]) == pytest.approx(speed0)
        assert state.velocity[2] == pytest.approx(0.0)


def test_coordinated_turn_position_stays_on_circle():
    speed = 5.0
    turn_rate_deg = 30.0
    trajectory = SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=6.0, turn_rate_deg=turn_rate_deg)],
        initial_velocity=[speed, 0.0, 0.0],
        gravity_magnitude=_G,
    )
    radius = speed / np.radians(turn_rate_deg)
    center = np.array([0.0, radius, 0.0])

    for t in [0.0, 1.0, 3.0, 6.0]:
        state = trajectory.state_at(t)
        assert np.linalg.norm(state.position - center) == pytest.approx(radius, abs=1e-9)


def test_coordinated_turn_attitude_tracks_heading_when_aligned():
    speed = 4.0
    turn_rate_deg = 25.0
    trajectory = SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=4.0, turn_rate_deg=turn_rate_deg)],
        initial_velocity=[speed, 0.0, 0.0],
        gravity_magnitude=_G,
    )

    for t in [0.0, 1.0, 2.0, 4.0]:
        state = trajectory.state_at(t)
        forward_quat = state.attitude * Quaternion(0, 1, 0, 0) * state.attitude.conjugate()
        forward = np.array([forward_quat.x, forward_quat.y, forward_quat.z])
        velocity_dir = state.velocity / np.linalg.norm(state.velocity)
        assert np.allclose(forward, velocity_dir, atol=1e-9)


def test_coordinated_turn_zero_rate_is_straight_line():
    trajectory = SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=2.0, turn_rate_deg=0.0)],
        initial_velocity=[2.0, -1.0, 0.5],
        gravity_magnitude=_G,
    )

    state = trajectory.state_at(2.0)
    assert np.allclose(state.velocity, [2.0, -1.0, 0.5])
    assert np.allclose(state.position, [4.0, -2.0, 1.0])
    assert np.allclose(state.accel_nav, [0, 0, 0])


def test_coordinated_turn_zero_initial_speed_stays_put():
    trajectory = SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=2.0, turn_rate_deg=45.0)], gravity_magnitude=_G
    )

    state = trajectory.state_at(2.0)
    assert np.allclose(state.velocity, [0, 0, 0])
    assert np.allclose(state.position, [0, 0, 0])


def test_coordinated_turn_chains_after_acceleration_segment():
    trajectory = SyntheticTrajectory(
        [
            MotionSegment(duration=2.0, accel_nav=[1.0, 0.0, 0.0]),
            CoordinatedTurnSegment(duration=3.0, turn_rate_deg=20.0),
        ],
        gravity_magnitude=_G,
    )

    boundary_state = trajectory.state_at(2.0)
    # Speed after 2s of accel=1 m/s^2 starting from rest: v=2 m/s along +x.
    assert np.allclose(boundary_state.velocity, [2.0, 0.0, 0.0])

    later_state = trajectory.state_at(3.5)
    assert np.hypot(later_state.velocity[0], later_state.velocity[1]) == pytest.approx(2.0)
