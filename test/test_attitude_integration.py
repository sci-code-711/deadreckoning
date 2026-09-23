import math

import numpy as np
import pytest

from deadrec.attitude_integration import (
    AdamsBashforth2Integrator,
    ConingIntegrator,
    EulerIntegrator,
    ExactExponentialIntegrator,
    MagnusIntegrator,
    MuntheKaasIntegrator,
    RK4Integrator,
)
from deadrec.interpolation import TwoPointLinearInterpolator
from deadrec.kinematics import omega_matrix, rk4_attitude_step
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


# --- EulerIntegrator ---


def test_euler_integrator_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = EulerIntegrator().integrate(qi, lambda t: np.zeros(3), t0=0.0, t1=0.05)

    _assert_quaternions_close(result, qi)


def test_euler_integrator_returns_unit_quaternion():
    result = EulerIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([1.0, 2.0, 3.0]), t0=0.0, t1=0.02
    )

    assert abs(result) == pytest.approx(1.0)


def test_euler_integrator_queries_omega_once_at_start():
    calls = []

    def omega(t):
        calls.append(t)
        return np.zeros(3)

    EulerIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.1)

    assert calls == [0.0]


def test_euler_integrator_error_shrinks_as_dt_shrinks():
    # First-order accuracy: halving dt should roughly halve the error
    # against the exact closed-form single-axis rotation.
    wz_rad = math.radians(90.0)

    def closed_form(dt):
        return Quaternion.from_axis_angle([0, 0, 1], wz_rad * dt)

    def error(dt):
        result = EulerIntegrator().integrate(
            Quaternion(1, 0, 0, 0), lambda t: np.array([0, 0, wz_rad]), t0=0.0, t1=dt
        )
        expected = closed_form(dt)
        return abs(result.z - expected.z)

    err_large = error(0.1)
    err_small = error(0.05)

    assert err_small < err_large * 0.6  # roughly halves, with slack


# --- ExactExponentialIntegrator ---


def test_exact_exponential_integrator_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = ExactExponentialIntegrator().integrate(qi, lambda t: np.zeros(3), t0=0.0, t1=0.05)

    _assert_quaternions_close(result, qi)


def test_exact_exponential_integrator_returns_unit_quaternion():
    result = ExactExponentialIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([1.0, 2.0, 3.0]), t0=0.0, t1=0.02
    )

    assert abs(result) == pytest.approx(1.0)


def test_exact_exponential_integrator_matches_closed_form_single_axis_rotation():
    wz_deg = 90.0
    dt = 0.37  # deliberately not small - this integrator should be exact regardless
    wz_rad = math.radians(wz_deg)

    result = ExactExponentialIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([0.0, 0.0, wz_rad]), t0=0.0, t1=dt
    )
    expected = Quaternion.from_axis_angle([0, 0, 1], wz_rad * dt)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_exact_exponential_integrator_is_exact_for_constant_multi_axis_rate():
    w = np.array([0.4, -0.9, 1.3])
    dt = 0.6

    result = ExactExponentialIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: w, t0=0.0, t1=dt
    )
    expected = Quaternion.from_axis_angle(w, np.linalg.norm(w) * dt)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_exact_exponential_integrator_queries_omega_once_at_midpoint():
    calls = []

    def omega(t):
        calls.append(t)
        return np.zeros(3)

    ExactExponentialIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.1)

    assert calls == [0.05]


# --- MuntheKaasIntegrator ---


def test_munthe_kaas_integrator_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = MuntheKaasIntegrator().integrate(qi, lambda t: np.zeros(3), t0=0.0, t1=0.05)

    _assert_quaternions_close(result, qi)


def test_munthe_kaas_integrator_returns_unit_quaternion():
    result = MuntheKaasIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([1.0, 2.0, 3.0]), t0=0.0, t1=0.02
    )

    assert abs(result) == pytest.approx(1.0)


def test_munthe_kaas_integrator_is_exact_for_constant_single_axis_rotation():
    wz_deg = 90.0
    dt = 0.37
    wz_rad = math.radians(wz_deg)

    result = MuntheKaasIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([0.0, 0.0, wz_rad]), t0=0.0, t1=dt
    )
    expected = Quaternion.from_axis_angle([0, 0, 1], wz_rad * dt)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_munthe_kaas_integrator_is_exact_for_a_linearly_varying_single_axis_rate():
    # Simpson's rule is exact for any polynomial up to cubic - a linearly
    # varying rate should integrate exactly too, unlike the constant-rate
    # case alone.
    dt = 0.4

    def omega(t):
        return np.array([0.0, 0.0, 2.0 + 5.0 * t])  # linear in t

    result = MuntheKaasIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=dt)

    true_angle = 2.0 * dt + 2.5 * dt**2  # integral of (2 + 5t) dt from 0 to dt
    expected = Quaternion.from_axis_angle([0, 0, 1], true_angle)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_munthe_kaas_integrator_queries_omega_at_start_midpoint_and_end():
    calls = []

    def omega(t):
        calls.append(t)
        return np.zeros(3)

    MuntheKaasIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.1)

    assert calls == [0.0, 0.05, 0.1]


# --- MagnusIntegrator ---


def test_magnus_integrator_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = MagnusIntegrator().integrate(qi, lambda t: np.zeros(3), t0=0.0, t1=0.05)

    _assert_quaternions_close(result, qi)


def test_magnus_integrator_returns_unit_quaternion():
    result = MagnusIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([1.0, 2.0, 3.0]), t0=0.0, t1=0.02
    )

    assert abs(result) == pytest.approx(1.0)


def test_magnus_integrator_is_exact_for_constant_single_axis_rotation():
    # Parallel endpoint vectors -> zero commutator term -> reduces to the
    # plain trapezoidal (exact, for constant rate) single-axis case.
    wz_deg = 90.0
    dt = 0.37
    wz_rad = math.radians(wz_deg)

    result = MagnusIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([0.0, 0.0, wz_rad]), t0=0.0, t1=dt
    )
    expected = Quaternion.from_axis_angle([0, 0, 1], wz_rad * dt)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_magnus_integrator_queries_omega_at_start_and_end():
    calls = []

    def omega(t):
        calls.append(t)
        return np.zeros(3)

    MagnusIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.1)

    assert calls == [0.0, 0.1]


def test_magnus_integrator_more_accurate_than_munthe_kaas_for_coning_case():
    # A case where the rate's *direction* genuinely changes within the
    # step (two orthogonal endpoint vectors) - the regime coning error
    # (and this integrator's commutator correction) actually shows up in.
    # A constant/parallel-vector rate can't distinguish these integrators
    # at all, since the commutator term is exactly zero either way.
    a = np.array([1.5, 0.0, 0.0])
    b = np.array([0.0, 1.8, 0.0])
    dt = 0.3

    def omega(t):
        frac = t / dt
        return a + frac * (b - a)

    # Fine-grained numerical reference: subdivide into many substeps and
    # integrate the same linearly-interpolated omega(t) exactly per
    # substep - converges to the true ODE solution under this omega(t).
    reference = Quaternion(1, 0, 0, 0)
    steps = np.linspace(0.0, dt, 2001)
    for i in range(len(steps) - 1):
        reference = ExactExponentialIntegrator().integrate(reference, omega, steps[i], steps[i + 1])

    def quaternion_error(q):
        diff = np.array([q.w, q.x, q.y, q.z]) - np.array(
            [reference.w, reference.x, reference.y, reference.z]
        )
        diff_flipped = np.array([q.w, q.x, q.y, q.z]) + np.array(
            [reference.w, reference.x, reference.y, reference.z]
        )
        return min(np.linalg.norm(diff), np.linalg.norm(diff_flipped))

    magnus_result = MagnusIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)
    munthe_kaas_result = MuntheKaasIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)

    magnus_error = quaternion_error(magnus_result)
    munthe_kaas_error = quaternion_error(munthe_kaas_result)

    assert magnus_error < munthe_kaas_error / 5


# --- ConingIntegrator ---


def test_coning_integrator_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = ConingIntegrator().integrate(qi, lambda t: np.zeros(3), t0=0.0, t1=0.05)

    _assert_quaternions_close(result, qi)


def test_coning_integrator_returns_unit_quaternion():
    result = ConingIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([1.0, 2.0, 3.0]), t0=0.0, t1=0.02
    )

    assert abs(result) == pytest.approx(1.0)


def test_coning_integrator_is_exact_for_constant_single_axis_rotation():
    wz_deg = 90.0
    dt = 0.37
    wz_rad = math.radians(wz_deg)

    result = ConingIntegrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([0.0, 0.0, wz_rad]), t0=0.0, t1=dt
    )
    expected = Quaternion.from_axis_angle([0, 0, 1], wz_rad * dt)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_coning_integrator_queries_omega_at_start_midpoint_and_end():
    calls = []

    def omega(t):
        calls.append(t)
        return np.zeros(3)

    ConingIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.1)

    assert calls == [0.0, 0.05, 0.1]


def test_coning_integrator_matches_magnus_when_midpoint_is_the_linear_average():
    # When w(t) is linear (so the midpoint sample equals the linear
    # average of the endpoints, giving the quadratic fit no genuine
    # curvature to exploit), ConingIntegrator's quadratic-based
    # commutator correction must reduce *exactly* to MagnusIntegrator's
    # linear one - this is the derivation's own correctness check,
    # kept here as a permanent regression test.
    a = np.array([1.5, -0.4, 0.2])
    b = np.array([0.0, 1.8, -0.6])
    dt = 0.3

    def omega(t):
        frac = t / dt
        return a + frac * (b - a)

    coning_result = ConingIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)
    magnus_result = MagnusIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)

    _assert_quaternions_close(coning_result, magnus_result, abs_tol=1e-12)


def test_coning_integrator_more_accurate_than_munthe_kaas_for_coning_case():
    # A genuinely rotating rate vector within the step (not just linearly
    # interpolated between two endpoints) - true coning motion, where the
    # midpoint sample carries real curvature information a 2-point linear
    # model (MagnusIntegrator) can't see, but ConingIntegrator's
    # quadratic fit can.
    # dt deliberately isn't a multiple of the coning period (which would
    # make a == b exactly, zeroing out one of the three correction terms
    # and understating the improvement) - a generic fraction of it instead.
    amplitude = 1.0
    omega_c = 2 * np.pi * 2.0  # 2 Hz
    dt = 0.15

    def omega(t):
        return amplitude * np.array([np.cos(omega_c * t), np.sin(omega_c * t), 0.0])

    # Fine-grained numerical reference: substep the *true* omega(t)
    # (not a linear interpolant) with ExactExponentialIntegrator.
    reference = Quaternion(1, 0, 0, 0)
    steps = np.linspace(0.0, dt, 1001)
    for i in range(len(steps) - 1):
        reference = ExactExponentialIntegrator().integrate(reference, omega, steps[i], steps[i + 1])

    def quaternion_error(q):
        diff = np.array([q.w, q.x, q.y, q.z]) - np.array(
            [reference.w, reference.x, reference.y, reference.z]
        )
        diff_flipped = np.array([q.w, q.x, q.y, q.z]) + np.array(
            [reference.w, reference.x, reference.y, reference.z]
        )
        return min(np.linalg.norm(diff), np.linalg.norm(diff_flipped))

    coning_result = ConingIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)
    munthe_kaas_result = MuntheKaasIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)
    magnus_result = MagnusIntegrator().integrate(Quaternion(1, 0, 0, 0), omega, 0.0, dt)

    coning_error = quaternion_error(coning_result)
    munthe_kaas_error = quaternion_error(munthe_kaas_result)
    magnus_error = quaternion_error(magnus_result)

    # Empirically ~4.4x and ~59x respectively for this case - safe margins
    # well under both, since the point is confirming the ordering holds,
    # not pinning the exact ratio.
    assert coning_error < munthe_kaas_error / 3
    assert coning_error < magnus_error / 10


# --- AdamsBashforth2Integrator ---


def test_adams_bashforth2_integrator_zero_rate_is_identity():
    qi = Quaternion.from_eul_angles(0.3, -0.2, 0.1)

    result = AdamsBashforth2Integrator().integrate(qi, lambda t: np.zeros(3), t0=0.0, t1=0.05)

    _assert_quaternions_close(result, qi)


def test_adams_bashforth2_integrator_returns_unit_quaternion():
    result = AdamsBashforth2Integrator().integrate(
        Quaternion(1, 0, 0, 0), lambda t: np.array([1.0, 2.0, 3.0]), t0=0.0, t1=0.02
    )

    assert abs(result) == pytest.approx(1.0)


def _bootstrap_step(q0, omega, t0, t1):
    """The plain forward-Euler formula AdamsBashforth2Integrator falls
    back to when it has no trustworthy history."""
    q0_vec = np.array([q0.w, q0.x, q0.y, q0.z])
    f0 = 0.5 * omega_matrix(omega(t0)) @ q0_vec
    q1_vec = q0_vec + (t1 - t0) * f0
    result = Quaternion(*q1_vec)
    return result * (1.0 / abs(result))


def test_adams_bashforth2_integrator_first_call_falls_back_to_bootstrap():
    q0 = Quaternion.from_eul_angles(0.1, 0.2, -0.3)

    def omega(t):
        return np.array([1.0, -0.5, 2.0])

    result = AdamsBashforth2Integrator().integrate(q0, omega, t0=0.0, t1=0.05)
    expected = _bootstrap_step(q0, omega, 0.0, 0.05)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_adams_bashforth2_integrator_uses_history_on_continuous_second_call():
    q0 = Quaternion.from_eul_angles(0.1, 0.2, -0.3)

    def omega(t):
        return np.array([1.0, -0.5, 2.0]) + t * np.array([0.2, 0.1, -0.3])

    integrator = AdamsBashforth2Integrator()
    q1 = integrator.integrate(q0, omega, t0=0.0, t1=0.05)
    q2 = integrator.integrate(q1, omega, t0=0.05, t1=0.10)

    # Hand-computed variable-step AB2 formula (here with equal steps).
    q0_vec = np.array([q0.w, q0.x, q0.y, q0.z])
    q1_vec = np.array([q1.w, q1.x, q1.y, q1.z])
    f_prev = 0.5 * omega_matrix(omega(0.0)) @ q0_vec
    f0 = 0.5 * omega_matrix(omega(0.05)) @ q1_vec
    h_prev = 0.05
    h = 0.05
    theta = h * (f0 * (1 + h / (2 * h_prev)) - f_prev * (h / (2 * h_prev)))
    expected_vec = q1_vec + theta
    expected = Quaternion(*expected_vec)
    expected = expected * (1.0 / abs(expected))

    _assert_quaternions_close(q2, expected, abs_tol=1e-12)

    # And it must differ from the plain bootstrap - confirms the history
    # path was actually taken, not silently ignored.
    bootstrap_only = _bootstrap_step(q1, omega, 0.05, 0.10)
    assert abs(q2.x - bootstrap_only.x) > 1e-6 or abs(q2.y - bootstrap_only.y) > 1e-6


def test_adams_bashforth2_integrator_falls_back_when_discontinuous():
    def omega(t):
        return np.array([1.0, -0.5, 2.0])

    integrator = AdamsBashforth2Integrator()
    integrator.integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.05)

    # A second call whose q0 doesn't match the first call's result - e.g.
    # a different, unrelated attitude at the same t0=0.05 - must not trust
    # the stored history.
    unrelated_q0 = Quaternion.from_eul_angles(0.5, 0.5, 0.5)
    result = integrator.integrate(unrelated_q0, omega, t0=0.05, t1=0.10)
    expected = _bootstrap_step(unrelated_q0, omega, 0.05, 0.10)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_adams_bashforth2_integrator_reset_clears_history():
    def omega(t):
        return np.array([1.0, -0.5, 2.0])

    integrator = AdamsBashforth2Integrator()
    q1 = integrator.integrate(Quaternion(1, 0, 0, 0), omega, t0=0.0, t1=0.05)
    integrator.reset()

    # Even though q1/t1=0.05 would otherwise be valid, continuing history,
    # reset() must have cleared it, so this call bootstraps instead.
    result = integrator.integrate(q1, omega, t0=0.05, t1=0.10)
    expected = _bootstrap_step(q1, omega, 0.05, 0.10)

    _assert_quaternions_close(result, expected, abs_tol=1e-12)


def test_adams_bashforth2_integrator_handles_variable_step_sizes():
    q0 = Quaternion.from_eul_angles(0.1, 0.2, -0.3)

    def omega(t):
        return np.array([1.0, -0.5, 2.0]) + t * np.array([0.2, 0.1, -0.3])

    integrator = AdamsBashforth2Integrator()
    q1 = integrator.integrate(q0, omega, t0=0.0, t1=0.05)  # h_prev = 0.05
    q2 = integrator.integrate(q1, omega, t0=0.05, t1=0.13)  # h = 0.08 (different)

    q0_vec = np.array([q0.w, q0.x, q0.y, q0.z])
    q1_vec = np.array([q1.w, q1.x, q1.y, q1.z])
    f_prev = 0.5 * omega_matrix(omega(0.0)) @ q0_vec
    f0 = 0.5 * omega_matrix(omega(0.05)) @ q1_vec
    h_prev = 0.05
    h = 0.08
    theta = h * (f0 * (1 + h / (2 * h_prev)) - f_prev * (h / (2 * h_prev)))
    expected_vec = q1_vec + theta
    expected = Quaternion(*expected_vec)
    expected = expected * (1.0 / abs(expected))

    _assert_quaternions_close(q2, expected, abs_tol=1e-12)


def test_adams_bashforth2_integrator_reduces_to_fixed_step_formula_when_uniform():
    # The derivation's own correctness check: with equal step sizes, the
    # variable-step formula must reduce exactly to the textbook
    # 1.5*f0 - 0.5*f_prev Adams-Bashforth-2 update.
    q0 = Quaternion.from_eul_angles(0.2, -0.1, 0.4)

    def omega(t):
        return np.array([0.5, 1.2, -0.7]) + t * np.array([-0.1, 0.3, 0.2])

    integrator = AdamsBashforth2Integrator()
    q1 = integrator.integrate(q0, omega, t0=0.0, t1=0.04)
    q2 = integrator.integrate(q1, omega, t0=0.04, t1=0.08)

    q1_vec = np.array([q1.w, q1.x, q1.y, q1.z])
    f_prev = 0.5 * omega_matrix(omega(0.0)) @ np.array([q0.w, q0.x, q0.y, q0.z])
    f0 = 0.5 * omega_matrix(omega(0.04)) @ q1_vec
    h = 0.04
    expected_vec = q1_vec + h * (1.5 * f0 - 0.5 * f_prev)
    expected = Quaternion(*expected_vec)
    expected = expected * (1.0 / abs(expected))

    _assert_quaternions_close(q2, expected, abs_tol=1e-12)


def test_adams_bashforth2_integrator_more_accurate_than_euler_after_warmup():
    # The multistep accuracy claim: at equal per-call cost (1 omega()
    # evaluation each), AdamsBashforth2Integrator should be clearly more
    # accurate than EulerIntegrator once a few small, uniform steps have
    # built up history - the asymptotic (small-step) regime multistep
    # methods are actually designed for.
    amplitude = 1.0
    omega_c = 2 * np.pi * 2.0
    dt = 0.02
    n_steps = 20

    def omega(t):
        return amplitude * np.array([np.cos(omega_c * t), np.sin(omega_c * t), 0.0])

    step_times = [i * dt for i in range(n_steps + 1)]

    reference = Quaternion(1, 0, 0, 0)
    for i in range(n_steps):
        substeps = np.linspace(step_times[i], step_times[i + 1], 1501)
        for j in range(len(substeps) - 1):
            reference = ExactExponentialIntegrator().integrate(
                reference, omega, substeps[j], substeps[j + 1]
            )

    def quaternion_error(q):
        diff = np.array([q.w, q.x, q.y, q.z]) - np.array(
            [reference.w, reference.x, reference.y, reference.z]
        )
        diff_flipped = np.array([q.w, q.x, q.y, q.z]) + np.array(
            [reference.w, reference.x, reference.y, reference.z]
        )
        return min(np.linalg.norm(diff), np.linalg.norm(diff_flipped))

    ab2 = AdamsBashforth2Integrator()
    q_ab2 = Quaternion(1, 0, 0, 0)
    q_euler = Quaternion(1, 0, 0, 0)
    euler = EulerIntegrator()
    for i in range(n_steps):
        q_ab2 = ab2.integrate(q_ab2, omega, step_times[i], step_times[i + 1])
        q_euler = euler.integrate(q_euler, omega, step_times[i], step_times[i + 1])

    ab2_error = quaternion_error(q_ab2)
    euler_error = quaternion_error(q_euler)

    # Empirically ~5.2x for this case - safe margin under it.
    assert ab2_error < euler_error / 2
