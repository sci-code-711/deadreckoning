from pathlib import Path

import numpy as np
import pytest

from deadrec.attitude import (
    attitude_aligning_vectors,
    estimate_gravity_magnitude,
    initial_attitude_from_gravity,
)
from deadrec.dead_reckoning import DeadReckoner
from deadrec.ekf import (
    GravityCorrectedEKF,
    WindowedGravityCorrectedEKF,
    _blend_attitudes,
    _heading_only,
)
from deadrec.interpolation import AngularRateInterpolator
from deadrec.quaternion import Quaternion
from deadrec.samples import ImuSample


def _approx_equal(a: Quaternion, b: Quaternion, tol=1e-8):
    return all(getattr(a, attr) == pytest.approx(getattr(b, attr), abs=tol) for attr in "wxyz")


def _approx_equal_up_to_sign(a: Quaternion, b: Quaternion, tol=1e-8):
    return _approx_equal(a, b, tol) or _approx_equal(a, -b, tol)


def test_heading_only_leaves_pure_heading_unchanged():
    yaw = Quaternion.from_eul_angles(0, 0, 0.7)

    assert _approx_equal(_heading_only(yaw), yaw)


def test_heading_only_discards_tilt():
    tilted = Quaternion.from_eul_angles(0.3, 0.4, 0.5)

    result = _heading_only(tilted)

    assert result.x == 0
    assert result.y == 0
    assert abs(result) == pytest.approx(1.0)
    assert result.w / result.z == pytest.approx(tilted.w / tilted.z)


def test_blend_attitudes_zero_weight_returns_first():
    a = Quaternion.from_eul_angles(0.1, 0.2, 0.3)
    b = Quaternion.from_eul_angles(-0.4, 0.1, 0.2)

    assert _approx_equal_up_to_sign(_blend_attitudes(a, b, weight_b=0.0), a)


def test_blend_attitudes_full_weight_returns_second():
    a = Quaternion.from_eul_angles(0.1, 0.2, 0.3)
    b = Quaternion.from_eul_angles(-0.4, 0.1, 0.2)

    assert _approx_equal_up_to_sign(_blend_attitudes(a, b, weight_b=1.0), b)


def test_gravity_corrected_ekf_skips_correction_when_deviation_is_large():
    initial = Quaternion(1, 0, 0, 0)
    s0 = ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0])
    s1 = ImuSample(t=0.1, accel=[5.0, 0.0, 9.8], gyro=[10.0, 0.0, 0.0])

    plain = DeadReckoner(initial, gravity_magnitude=9.8)
    plain.step(s0)
    plain_state = plain.step(s1)

    ekf = GravityCorrectedEKF(initial, gravity_magnitude=9.8)
    ekf.step(s0)
    ekf_state = ekf.step(s1)

    assert _approx_equal(ekf_state.attitude, plain_state.attitude)


def test_gravity_corrected_ekf_pulls_tilted_attitude_toward_level():
    # With zero gyro rate, a plain DeadReckoner can't correct a tilt error
    # at all - RK4 propagation leaves attitude exactly where it started.
    initial = Quaternion.from_eul_angles(0.1, 0, 0)
    s0 = ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0])
    s1 = ImuSample(t=0.1, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0])

    plain = DeadReckoner(initial, gravity_magnitude=9.8)
    plain.step(s0)
    plain_state = plain.step(s1)
    assert _approx_equal(plain_state.attitude, initial)

    # The tilt produces a large enough deviation that it needs a raised
    # threshold to engage the correction here.
    ekf = GravityCorrectedEKF(initial, gravity_magnitude=9.8, deviation_threshold=2.0)
    ekf.step(s0)
    ekf_state = ekf.step(s1)

    initial_pitch = initial.to_euler_angles()[0]
    ekf_pitch = ekf_state.attitude.to_euler_angles()[0]
    assert 0 < ekf_pitch < initial_pitch


class _NonCausalInterpolator(AngularRateInterpolator):
    """Minimal test double: declares look-ahead context it doesn't actually
    use, purely to exercise the causal/non-causal compatibility check. A
    real non-causal interpolator (CentredCubicHermiteInterpolator) is
    exercised end-to-end elsewhere."""

    context_after = 1

    def build(self, window, step_pos):
        start, end = window[step_pos - 1], window[step_pos]
        w0 = np.radians(start.gyro)
        w1 = np.radians(end.gyro)
        t0, t1 = start.t, end.t

        def omega(t):
            span = t1 - t0
            if span == 0:
                return w0
            frac = (t - t0) / span
            return w0 + frac * (w1 - w0)

        return omega


def test_gravity_corrected_ekf_rejects_non_causal_interpolator_at_construction():
    with pytest.raises(ValueError, match="_NonCausalInterpolator"):
        GravityCorrectedEKF(
            Quaternion(1, 0, 0, 0), gravity_magnitude=9.8, interpolator=_NonCausalInterpolator()
        )


def test_windowed_ekf_accepts_non_causal_interpolator():
    samples = [
        ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0]),
        ImuSample(t=0.1, accel=[0.1, 0.0, 9.8], gyro=[5.0, 0.0, 0.0]),
        ImuSample(t=0.2, accel=[0.0, 0.1, 9.8], gyro=[0.0, 5.0, 0.0]),
        ImuSample(t=0.3, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 5.0]),
    ]

    ekf = WindowedGravityCorrectedEKF(
        Quaternion(1, 0, 0, 0), gravity_magnitude=9.8, interpolator=_NonCausalInterpolator()
    )
    states = ekf.run(samples)

    assert len(states) == len(samples)
    for state in states:
        assert abs(state.attitude) == pytest.approx(1.0)


# --- Independent reference implementation, used only to regression-test
# GravityCorrectedEKF against real data (own RK4/Omega, eigenvector
# averaging and alignment code - doesn't call into deadrec.kinematics,
# deadrec.attitude or deadrec.ekf).


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


def _alignment(a, target):
    a = np.asarray(a, dtype=float)
    a = a / np.linalg.norm(a)
    cross = np.cross(a, target)
    cross_norm = np.linalg.norm(cross)

    if cross_norm == 0:
        return Quaternion(1, 0, 0, 0)

    axis = cross / cross_norm
    phi = np.arctan(cross_norm / np.dot(a, target))
    if phi < 0:
        phi = phi + np.pi

    return Quaternion(
        np.cos(phi / 2),
        axis[0] * np.sin(phi / 2),
        axis[1] * np.sin(phi / 2),
        axis[2] * np.sin(phi / 2),
    )


def _initial_attitude(accel_samples, g):
    n = len(accel_samples)
    mat = np.zeros((4, 4))

    for row in accel_samples:
        q = _alignment(row, g)
        qv = np.array([q.w, q.x, q.y, q.z])
        mat = mat + np.outer(qv, qv) / n

    eig_val, eig_vec = np.linalg.eig(mat)
    q = eig_vec[:, np.argmax(eig_val)].real
    return Quaternion(*(q / np.linalg.norm(q)))


def _heading_only_ref(q):
    norm = np.sqrt(q.w**2 + q.z**2)
    return Quaternion(q.w / norm, 0, 0, q.z / norm)


def _reference_ekf(
    t, accel, gyro, *, attitude_n, gravity_n, fix_y_axis_bug, deviation_threshold=0.02, beta=0.2
):
    """Reference port of EKF.py's per-row loop, with the same Y-axis bug
    (see test_dead_reckoning.py) optionally reproduced."""
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
        predicted = _rk4(qc, gyro[r - 1], gyro[r], dt)
        predicted_av = to_nav(accel[r], predicted)

        if np.linalg.norm(predicted_av) < deviation_threshold:
            heading = _heading_only_ref(predicted)
            gravity_estimate = heading * _alignment(accel[r], g)

            pv = np.array([predicted.w, predicted.x, predicted.y, predicted.z])
            gv = np.array(
                [gravity_estimate.w, gravity_estimate.x, gravity_estimate.y, gravity_estimate.z]
            )
            mat = np.outer(pv, pv) * (1 - beta) + np.outer(gv, gv) * beta
            eig_val, eig_vec = np.linalg.eig(mat)
            q = eig_vec[:, np.argmax(eig_val)].real
            qc = Quaternion(*(q / np.linalg.norm(q)))
            av = to_nav(accel[r], qc)
        else:
            qc = predicted
            av = predicted_av

        attitudes.append(qc)
        accel_nav[r] = av

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


def _run_gravity_corrected_ekf(t, accel, gyro, attitude_n, gravity_n):
    initial_attitude = initial_attitude_from_gravity(accel[:attitude_n])
    gravity_magnitude = estimate_gravity_magnitude(accel[:gravity_n])
    ekf = GravityCorrectedEKF(initial_attitude, gravity_magnitude)
    samples = [ImuSample(t=t[i], accel=accel[i], gyro=gyro[i]) for i in range(len(t))]
    return ekf.run(samples)


def test_gravity_corrected_ekf_matches_corrected_reference_on_example_data():
    t, accel, gyro = _load_example_data()
    attitude_n, gravity_n = 30, 400  # matches EKF.py's parameters

    states = _run_gravity_corrected_ekf(t, accel, gyro, attitude_n, gravity_n)
    ref_attitudes, ref_accel_nav, ref_velocity, ref_position = _reference_ekf(
        t, accel, gyro, attitude_n=attitude_n, gravity_n=gravity_n, fix_y_axis_bug=True
    )

    for i, state in enumerate(states):
        assert _approx_equal(state.attitude, ref_attitudes[i], tol=1e-6)
        assert np.allclose(state.accel_nav, ref_accel_nav[i], atol=1e-6)
        assert np.allclose(state.velocity, ref_velocity[i], atol=1e-6)
        assert np.allclose(state.position, ref_position[i], atol=1e-6)


def test_gravity_corrected_ekf_y_position_diverges_from_original_buggy_script():
    t, accel, gyro = _load_example_data()
    attitude_n, gravity_n = 30, 400

    states = _run_gravity_corrected_ekf(t, accel, gyro, attitude_n, gravity_n)
    _, _, _, buggy_position = _reference_ekf(
        t, accel, gyro, attitude_n=attitude_n, gravity_n=gravity_n, fix_y_axis_bug=False
    )

    final_y = states[-1].position[1]
    assert abs(final_y - buggy_position[-1, 1]) > 1e-3


# --- WindowedGravityCorrectedEKF ---


def test_windowed_ekf_step_is_disabled():
    ekf = WindowedGravityCorrectedEKF(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)

    with pytest.raises(NotImplementedError):
        ekf.step(ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0]))


def test_windowed_ekf_window_bounds_near_start_clips_to_zero():
    ekf = WindowedGravityCorrectedEKF(
        Quaternion(1, 0, 0, 0), gravity_magnitude=9.8, window_radius=5
    )

    assert ekf._window_bounds(2, n=100) == (0, 8)


def test_windowed_ekf_window_bounds_near_end_clips_to_n():
    ekf = WindowedGravityCorrectedEKF(
        Quaternion(1, 0, 0, 0), gravity_magnitude=9.8, window_radius=5
    )

    assert ekf._window_bounds(97, n=100) == (92, 100)


def test_windowed_ekf_window_bounds_in_middle_is_symmetric():
    ekf = WindowedGravityCorrectedEKF(
        Quaternion(1, 0, 0, 0), gravity_magnitude=9.8, window_radius=5
    )

    assert ekf._window_bounds(50, n=100) == (45, 56)


def test_windowed_ekf_run_empty_returns_empty():
    ekf = WindowedGravityCorrectedEKF(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)

    assert ekf.run([]) == []


def test_windowed_ekf_run_single_sample_seeds_only():
    ekf = WindowedGravityCorrectedEKF(Quaternion(1, 0, 0, 0), gravity_magnitude=9.8)
    sample = ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0])

    states = ekf.run([sample])

    assert len(states) == 1
    assert np.allclose(states[0].velocity, [0.0, 0.0, 0.0])
    assert np.allclose(states[0].position, [0.0, 0.0, 0.0])


def _reference_windowed_ekf(
    t,
    accel,
    gyro,
    *,
    attitude_n,
    gravity_n,
    fix_y_axis_bug,
    fix_edge_weight_bug,
    window_radius=5,
    deviation_threshold=0.035,
    beta=0.2,
):
    """Reference port of EKF_fut.py's per-row loop, reusing the independent
    _rk4/_alignment/_heading_only_ref/_initial_attitude helpers above (not
    calling into deadrec.kinematics/deadrec.attitude/deadrec.ekf), with the
    same Y-axis bug and edge-window-weighting bug optionally reproduced.

    EKF_fut.py always divides its windowed gravity estimate by the full
    2 * window_radius + 1, even when the window is clipped near either end
    of the sample sequence and has fewer actual terms - so a clipped
    window's estimate carries proportionally less weight than an interior
    one instead of being renormalised. fix_edge_weight_bug=True divides by
    the actual window length instead."""
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

    fixed_window_size = 2 * window_radius + 1

    for r in range(1, n):
        qi = qc
        dt = t[r] - t[r - 1]

        if r < window_radius:
            rmin, rmax = 0, r + window_radius + 1
        elif r > n - (window_radius + 1):
            rmin, rmax = r - window_radius, n
        else:
            rmin, rmax = r - window_radius, r + window_radius + 1
        rmax = min(rmax, n)

        deviations = [np.linalg.norm(accel_nav[row]) for row in range(rmin, r)]

        av_pred_at_r = None
        for row in range(r, rmax):
            q_pred = _rk4(qi, gyro[row - 1], gyro[row], dt)
            av_pred = to_nav(accel[row], q_pred)
            if row == r:
                av_pred_at_r = av_pred
                qc = q_pred
            deviations.append(np.linalg.norm(av_pred))

        if max(deviations) < deviation_threshold:
            qhead = _heading_only_ref(qc)
            divisor = (rmax - rmin) if fix_edge_weight_bug else fixed_window_size
            gmat = np.zeros((4, 4))
            for row in range(rmin, rmax):
                qg = qhead * _alignment(accel[row], g)
                qgv = np.array([qg.w, qg.x, qg.y, qg.z])
                gmat = gmat + np.outer(qgv, qgv) / divisor

            qcv = np.array([qc.w, qc.x, qc.y, qc.z])
            mat = np.outer(qcv, qcv) * (1 - beta) + gmat * beta
            eig_val, eig_vec = np.linalg.eig(mat)
            q = eig_vec[:, np.argmax(eig_val)].real
            qc = Quaternion(*(q / np.linalg.norm(q)))
            av = to_nav(accel[r], qc)
        else:
            av = av_pred_at_r

        attitudes.append(qc)
        accel_nav[r] = av

        velocity[r] = velocity[r - 1] + (accel_nav[r - 1] + accel_nav[r]) * dt / 2

        x_terms = (accel_nav[r - 1, 0], accel_nav[r, 0])
        y_terms = (accel_nav[r - 1, 1], accel_nav[r, 1]) if fix_y_axis_bug else x_terms
        z_terms = (accel_nav[r - 1, 2], accel_nav[r, 2])

        position[r, 0] = position[r - 1, 0] + velocity[r - 1, 0] * dt + sum(x_terms) * dt**2 / 4
        position[r, 1] = position[r - 1, 1] + velocity[r - 1, 1] * dt + sum(y_terms) * dt**2 / 4
        position[r, 2] = position[r - 1, 2] + velocity[r - 1, 2] * dt + sum(z_terms) * dt**2 / 4

    return attitudes, accel_nav, velocity, position


def _run_windowed_ekf(t, accel, gyro, attitude_n, gravity_n):
    initial_attitude = initial_attitude_from_gravity(accel[:attitude_n])
    gravity_magnitude = estimate_gravity_magnitude(accel[:gravity_n])
    ekf = WindowedGravityCorrectedEKF(initial_attitude, gravity_magnitude)
    samples = [ImuSample(t=t[i], accel=accel[i], gyro=gyro[i]) for i in range(len(t))]
    return ekf.run(samples)


def test_windowed_ekf_matches_corrected_reference_on_example_data():
    t, accel, gyro = _load_example_data()
    attitude_n, gravity_n = 30, 300  # matches EKF_fut.py's parameters

    states = _run_windowed_ekf(t, accel, gyro, attitude_n, gravity_n)
    ref_attitudes, ref_accel_nav, ref_velocity, ref_position = _reference_windowed_ekf(
        t,
        accel,
        gyro,
        attitude_n=attitude_n,
        gravity_n=gravity_n,
        fix_y_axis_bug=True,
        fix_edge_weight_bug=True,
    )

    for i, state in enumerate(states):
        assert _approx_equal(state.attitude, ref_attitudes[i], tol=1e-6)
        assert np.allclose(state.accel_nav, ref_accel_nav[i], atol=1e-6)
        assert np.allclose(state.velocity, ref_velocity[i], atol=1e-6)
        assert np.allclose(state.position, ref_position[i], atol=1e-6)


def test_windowed_ekf_y_position_diverges_from_original_buggy_script():
    t, accel, gyro = _load_example_data()
    attitude_n, gravity_n = 30, 300

    states = _run_windowed_ekf(t, accel, gyro, attitude_n, gravity_n)
    _, _, _, buggy_position = _reference_windowed_ekf(
        t,
        accel,
        gyro,
        attitude_n=attitude_n,
        gravity_n=gravity_n,
        fix_y_axis_bug=False,
        fix_edge_weight_bug=False,
    )

    final_y = states[-1].position[1]
    assert abs(final_y - buggy_position[-1, 1]) > 1e-3


def test_windowed_ekf_normalises_gravity_estimate_for_clipped_windows():
    # example_data/Example_data.csv doesn't happen to exercise a clipped
    # window where the correction actually engages, so this uses a short
    # synthetic, low-noise, zero-gyro sequence where it reliably does:
    # with no rotation, the RK4-predicted attitude never changes, so the
    # deviation gate stays open and every step gets corrected.
    rng = np.random.default_rng(0)
    n = 7
    window_radius = 2
    t = np.arange(n, dtype=float) * 0.1
    accel = np.tile([0.0, 0.0, 9.8], (n, 1))
    accel[:, 0] += rng.normal(scale=0.3, size=n)
    gyro = np.zeros((n, 3))
    samples = [ImuSample(t=t[i], accel=accel[i], gyro=gyro[i]) for i in range(n)]

    initial_attitude = Quaternion(1, 0, 0, 0)
    gravity_magnitude = 9.8

    ekf = WindowedGravityCorrectedEKF(
        initial_attitude,
        gravity_magnitude,
        deviation_threshold=5.0,
        window_radius=window_radius,
    )
    states = ekf.run(samples)

    # Recompute the r=1 correction (window [0, 4) - clipped, since
    # window_radius=2 would otherwise reach back to index -1) using the
    # pre-fix normalisation, which always divided by the full
    # 2 * window_radius + 1 regardless of how many terms were actually
    # summed.
    r = 1
    rmin, rmax = 0, r + window_radius + 1
    heading = _heading_only(initial_attitude)  # zero gyro => prediction == initial_attitude
    fixed_divisor = 2 * window_radius + 1
    old_gravity_mat = np.zeros((4, 4))
    for row in range(rmin, rmax):
        qg = heading * attitude_aligning_vectors(accel[row], (0, 0, 1))
        qgv = np.array([qg.w, qg.x, qg.y, qg.z])
        old_gravity_mat = old_gravity_mat + np.outer(qgv, qgv) / fixed_divisor

    predicted_v = np.array([heading.w, heading.x, heading.y, heading.z])
    old_mat = np.outer(predicted_v, predicted_v) * (1 - ekf.beta) + old_gravity_mat * ekf.beta
    eig_val, eig_vec = np.linalg.eig(old_mat)
    q = eig_vec[:, np.argmax(eig_val)].real
    old_attitude = Quaternion(*(q / np.linalg.norm(q)))

    assert not _approx_equal(states[r].attitude, old_attitude, tol=1e-9)
