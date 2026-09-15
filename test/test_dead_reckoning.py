from pathlib import Path

import numpy as np
import pytest

from deadrec.attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from deadrec.dead_reckoning import DeadReckoner, accel_to_nav_frame
from deadrec.quaternion import Quaternion
from deadrec.samples import ImuSample


def test_accel_to_nav_frame_identity_attitude_subtracts_gravity():
    result = accel_to_nav_frame([0.1, 0.2, 9.8], Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)

    assert np.allclose(result, [0.1, 0.2, 0.0])


def test_accel_to_nav_frame_rotates_before_subtracting_gravity():
    # This rotation maps body +x onto nav +z.
    attitude = Quaternion.from_eul_angles(0, -np.pi / 4, 0)

    result = accel_to_nav_frame([9.8, 0.0, 0.0], attitude, gravity_magnitude=9.8)

    assert np.allclose(result, [0.0, 0.0, 0.0], atol=1e-8)


def test_dead_reckoner_first_step_seeds_zero_velocity_and_position():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)
    sample = ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0])

    state = reckoner.step(sample)

    assert state.attitude == Quaternion(1, 0, 0, 0)
    assert np.allclose(state.accel_nav, [0.0, 0.0, 0.0])
    assert np.allclose(state.velocity, [0.0, 0.0, 0.0])
    assert np.allclose(state.position, [0.0, 0.0, 0.0])


def test_dead_reckoner_first_step_seeds_given_velocity_and_position():
    reckoner = DeadReckoner(
        Quaternion(1, 0, 0, 0),
        gravity_magnitude=9.8,
        initial_velocity=[1.0, -2.0, 0.5],
        initial_position=[10.0, 20.0, 30.0],
    )
    sample = ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0])

    state = reckoner.step(sample)

    assert np.allclose(state.velocity, [1.0, -2.0, 0.5])
    assert np.allclose(state.position, [10.0, 20.0, 30.0])


def test_dead_reckoner_can_resume_from_a_previous_run_final_state():
    samples = [
        ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0]),
        ImuSample(t=0.1, accel=[0.2, 0.0, 9.8], gyro=[5.0, 0.0, 0.0]),
        ImuSample(t=0.2, accel=[0.0, 0.3, 9.8], gyro=[0.0, 5.0, 0.0]),
        ImuSample(t=0.3, accel=[0.1, 0.1, 9.8], gyro=[0.0, 0.0, 5.0]),
    ]

    continuous_states = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8).run(samples)

    # Run the first two samples, then hand off to a fresh DeadReckoner
    # seeded from that run's final attitude/velocity/position, rather than
    # from a zeroed-out "known good state".
    first_reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)
    handoff = first_reckoner.run(samples[:2])[-1]

    resumed_reckoner = DeadReckoner(
        handoff.attitude,
        gravity_magnitude=9.8,
        initial_velocity=handoff.velocity,
        initial_position=handoff.position,
    )
    # A DeadReckoner's own first step always re-seeds rather than
    # propagating, so replay the sample the handoff state came from before
    # continuing with genuinely new samples.
    resumed_reckoner.step(samples[1])
    resumed_states = resumed_reckoner.run(samples[2:])

    for expected, actual in zip(continuous_states[2:], resumed_states):
        assert np.allclose(actual.position, expected.position)
        assert np.allclose(actual.velocity, expected.velocity)


def test_dead_reckoner_integrates_constant_acceleration():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=0.0)
    reckoner.step(ImuSample(t=0.0, accel=[1.0, 0.0, 0.0], gyro=[0.0, 0.0, 0.0]))
    state = reckoner.step(ImuSample(t=1.0, accel=[1.0, 0.0, 0.0], gyro=[0.0, 0.0, 0.0]))

    # Constant acceleration a=1 over dt=1: v = a*t = 1, x = 0.5*a*t^2 = 0.5.
    assert np.allclose(state.velocity, [1.0, 0.0, 0.0])
    assert np.allclose(state.position, [0.5, 0.0, 0.0])


def test_dead_reckoner_run_matches_manual_steps():
    samples = [
        ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0]),
        ImuSample(t=0.1, accel=[0.1, 0.0, 9.8], gyro=[5.0, 0.0, 0.0]),
        ImuSample(t=0.2, accel=[0.0, 0.1, 9.8], gyro=[0.0, 5.0, 0.0]),
    ]

    run_states = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8).run(samples)

    manual_reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)
    manual_states = [manual_reckoner.step(s) for s in samples]

    for run_state, manual_state in zip(run_states, manual_states):
        assert run_state.attitude == manual_state.attitude
        assert np.allclose(run_state.position, manual_state.position)
        assert np.allclose(run_state.velocity, manual_state.velocity)


# --- Independent reference implementation, used only to regression-test
# DeadReckoner against real data. Reimplements the algorithm from first
# principles (its own RK4/Omega and eigenvector-averaging code) rather than
# calling deadrec.kinematics/deadrec.attitude, so it doesn't just check the
# implementation against itself.


def _omega(w):
    return np.array(
        [
            [0, -w[0], -w[1], -w[2]],
            [w[0], 0, w[2], -w[1]],
            [w[1], -w[2], 0, w[0]],
            [w[2], w[1], -w[0], 0],
        ]
    )


def _rk4(qi, w1, w2, dt):
    qt = np.array([qi.w, qi.x, qi.y, qi.z])
    w11 = np.radians(np.asarray(w1, dtype=float))
    w22 = np.radians(np.asarray(w2, dtype=float))

    q1 = qt
    k1 = 0.5 * _omega(w11) @ q1
    q2 = qt + dt * 0.5 * k1
    k2 = 0.5 * _omega(0.5 * (w11 + w22)) @ q2
    q3 = qt + dt * 0.5 * k2
    k3 = 0.5 * _omega(0.5 * (w11 + w22)) @ q3
    q4 = qt + dt * k3
    k4 = 0.5 * _omega(w22) @ q4

    qf = qt + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return Quaternion(*(qf / np.linalg.norm(qf)))


def _initial_attitude(accel_samples, g):
    n = len(accel_samples)
    mat = np.zeros((4, 4))

    for row in accel_samples:
        t = np.asarray(row, dtype=float)
        t = t / np.linalg.norm(t)
        cross = np.cross(t, g)
        cross_norm = np.linalg.norm(cross)

        if cross_norm == 0:
            q = [1, 0, 0, 0]
        else:
            axis = cross / cross_norm
            phi = np.arctan(cross_norm / np.dot(t, g))
            if phi < 0:
                phi = phi + np.pi
            q = [
                np.cos(phi / 2),
                axis[0] * np.sin(phi / 2),
                axis[1] * np.sin(phi / 2),
                axis[2] * np.sin(phi / 2),
            ]

        mat = mat + np.outer(q, q) / n

    eig_val, eig_vec = np.linalg.eig(mat)
    q = eig_vec[:, np.argmax(eig_val)].real
    return Quaternion(*(q / np.linalg.norm(q)))


def _reference_dead_reckoning(t, accel, gyro, *, attitude_n, gravity_n, fix_y_axis_bug):
    """Reference port of the original per-row integration loop. The
    original scripts' Y-position update reused the X-axis acceleration
    term instead of the Y-axis one; pass fix_y_axis_bug=False to reproduce
    that bug verbatim, or True for the corrected version DeadReckoner
    implements."""
    g = np.array([0, 0, 1])
    n = len(t)

    gravity_magnitude = np.mean(np.linalg.norm(accel[:gravity_n], axis=1))
    qc = _initial_attitude(accel[:attitude_n], g)

    def to_nav(a, q):
        aq = Quaternion(0, *a)
        rotated = q * aq * q.conjugate()
        return np.array([rotated.x, rotated.y, rotated.z]) - gravity_magnitude * g

    attitudes = [qc]
    accel_nav = np.zeros((n, 3))
    velocity = np.zeros((n, 3))
    position = np.zeros((n, 3))
    accel_nav[0] = to_nav(accel[0], qc)

    for r in range(1, n):
        dt = t[r] - t[r - 1]
        qc = _rk4(qc, gyro[r - 1], gyro[r], dt)
        attitudes.append(qc)
        accel_nav[r] = to_nav(accel[r], qc)

        velocity[r] = velocity[r - 1] + (accel_nav[r - 1] + accel_nav[r]) * dt / 2

        x_terms = (accel_nav[r - 1, 0], accel_nav[r, 0])
        y_terms = (accel_nav[r - 1, 1], accel_nav[r, 1]) if fix_y_axis_bug else x_terms
        z_terms = (accel_nav[r - 1, 2], accel_nav[r, 2])

        position[r, 0] = position[r - 1, 0] + velocity[r - 1, 0] * dt + sum(x_terms) * dt**2 / 4
        position[r, 1] = position[r - 1, 1] + velocity[r - 1, 1] * dt + sum(y_terms) * dt**2 / 4
        position[r, 2] = position[r - 1, 2] + velocity[r - 1, 2] * dt + sum(z_terms) * dt**2 / 4

    return attitudes, accel_nav, velocity, position


def _load_example_data():
    path = Path(__file__).resolve().parent.parent / "example_data" / "Example_data.csv"
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["t"]
    accel = np.column_stack([data["ax"], data["ay"], data["az"]])
    gyro = np.column_stack([data["vl"], data["vm"], data["vn"]])
    return t, accel, gyro


def _run_dead_reckoner(t, accel, gyro, attitude_n, gravity_n):
    initial_attitude = initial_attitude_from_gravity(accel[:attitude_n])
    gravity_magnitude = estimate_gravity_magnitude(accel[:gravity_n])
    reckoner = DeadReckoner(initial_attitude, gravity_magnitude)
    samples = [ImuSample(t=t[i], accel=accel[i], gyro=gyro[i]) for i in range(len(t))]
    return reckoner.run(samples)


def test_dead_reckoner_matches_corrected_reference_on_example_data():
    t, accel, gyro = _load_example_data()
    attitude_n, gravity_n = 60, 400  # matches the original script's parameters

    states = _run_dead_reckoner(t, accel, gyro, attitude_n, gravity_n)
    ref_attitudes, ref_accel_nav, ref_velocity, ref_position = _reference_dead_reckoning(
        t, accel, gyro, attitude_n=attitude_n, gravity_n=gravity_n, fix_y_axis_bug=True
    )

    for i, state in enumerate(states):
        ref_q = ref_attitudes[i]
        assert state.attitude.w == pytest.approx(ref_q.w, abs=1e-6)
        assert state.attitude.x == pytest.approx(ref_q.x, abs=1e-6)
        assert state.attitude.y == pytest.approx(ref_q.y, abs=1e-6)
        assert state.attitude.z == pytest.approx(ref_q.z, abs=1e-6)
        assert np.allclose(state.accel_nav, ref_accel_nav[i], atol=1e-6)
        assert np.allclose(state.velocity, ref_velocity[i], atol=1e-6)
        assert np.allclose(state.position, ref_position[i], atol=1e-6)


def test_dead_reckoner_y_position_diverges_from_original_buggy_script():
    # Demonstrates the Y-position fix is real: DeadReckoner's Y position
    # should NOT match what the original bug (reusing the X-axis
    # acceleration term) would have produced.
    t, accel, gyro = _load_example_data()
    attitude_n, gravity_n = 60, 400

    states = _run_dead_reckoner(t, accel, gyro, attitude_n, gravity_n)
    _, _, _, buggy_position = _reference_dead_reckoning(
        t, accel, gyro, attitude_n=attitude_n, gravity_n=gravity_n, fix_y_axis_bug=False
    )

    final_y = states[-1].position[1]
    assert abs(final_y - buggy_position[-1, 1]) > 1e-3
