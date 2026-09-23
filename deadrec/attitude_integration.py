"""Pluggable attitude-integration strategies.

An :class:`AttitudeIntegrator` propagates a quaternion attitude across a
step using an angular-rate function assembled by a
:class:`deadrec.interpolation.AngularRateInterpolator`. The two are
orthogonal: an integrator queries ``omega(t)`` at whatever times its scheme
needs, and doesn't know (or care) how many times it'll be queried or how
that function was built.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from .kinematics import omega_matrix
from .quaternion import Quaternion


class AttitudeIntegrator(ABC):
    """Propagates attitude across one step given a continuous angular-rate
    function."""

    @abstractmethod
    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        """
        Integrate attitude from ``t0`` to ``t1``.

        Args:
            * q0 {``Quaternion``} -- Attitude at ``t0``.
            * omega {``Callable[[float], np.ndarray]``} -- Body-frame
              angular rate (rad/s) as a function of time, e.g. from
              :meth:`deadrec.interpolation.AngularRateInterpolator.build`.
            * t0 {``float``} -- Start time of the step (s).
            * t1 {``float``} -- End time of the step (s).

        Returns:
            * {``Quaternion``} -- Attitude at ``t1``.

        """

    def reset(self) -> None:
        """
        Clear any internal state carried between :meth:`integrate` calls.

        A no-op for every stateless (single-step) integrator - overridden
        by stateful ones (see :class:`AdamsBashforth2Integrator`) that
        carry history across calls. Call this before reusing one
        integrator instance for a second, independent sequence of steps,
        so the new sequence doesn't see history left over from the first.

        """


class RK4Integrator(AttitudeIntegrator):
    """
    Classical 4-stage Runge-Kutta, with the same stage structure as
    :func:`deadrec.kinematics.rk4_attitude_step`, generalised to query
    ``omega(t)`` at the step's start, midpoint and end instead of linearly
    blending two fixed endpoint vectors internally. It reuses
    :func:`deadrec.kinematics.omega_matrix` for its stage math but is a
    parallel implementation, not a wrapper around
    :func:`deadrec.kinematics.rk4_attitude_step` - wrapping it would leak
    interpolator internals into the integrator.
    """

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        dt = t1 - t0
        tm = 0.5 * (t0 + t1)
        qt = np.array([q0.w, q0.x, q0.y, q0.z])

        q1 = qt
        k1 = 0.5 * omega_matrix(omega(t0)) @ q1
        q2 = qt + dt * 0.5 * k1
        k2 = 0.5 * omega_matrix(omega(tm)) @ q2
        q3 = qt + dt * 0.5 * k2
        k3 = 0.5 * omega_matrix(omega(tm)) @ q3
        q4 = qt + dt * k3
        k4 = 0.5 * omega_matrix(omega(t1)) @ q4

        qf = qt + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        qf = qf / np.linalg.norm(qf)

        return Quaternion(*qf)


class EulerIntegrator(AttitudeIntegrator):
    """
    First-order (forward Euler) integration: a single ``omega(t0)``
    evaluation, then one linear step of the ODE, renormalized. The cheapest
    strategy available - one ``omega()`` call and one 4x4 matrix-vector
    product per step - useful as a low-cost, low-accuracy baseline to
    compare other integrators against.
    """

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        dt = t1 - t0
        qt = np.array([q0.w, q0.x, q0.y, q0.z])

        k1 = 0.5 * omega_matrix(omega(t0)) @ qt
        qf = qt + dt * k1
        qf = qf / np.linalg.norm(qf)

        return Quaternion(*qf)


class ExactExponentialIntegrator(AttitudeIntegrator):
    """
    Closed-form quaternion-exponential update via
    :meth:`Quaternion.from_axis_angle`, exact when the angular rate is
    genuinely constant over the step. Samples ``omega()`` once, at the
    step's midpoint - the best single-point estimate when the rate isn't
    quite constant, and it doesn't matter which point is sampled when the
    rate genuinely is constant (which is where this integrator is most
    useful, e.g. paired with
    :class:`deadrec.interpolation.ZeroOrderHoldInterpolator`).
    """

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        dt = t1 - t0
        w = omega(0.5 * (t0 + t1))
        angle = np.linalg.norm(w) * dt

        result = q0 * Quaternion.from_axis_angle(w, angle)
        return result * (1.0 / abs(result))


class MuntheKaasIntegrator(AttitudeIntegrator):
    """
    Integrates the angular rate itself - via classical RK4 quadrature,
    which for a rate-only vector ODE reduces exactly to Simpson's rule:
    ``(dt/6) * (omega(t0) + 4*omega(tm) + omega(t1))`` - to get a single net
    rotation vector for the whole step, then composes it onto ``q0`` with
    one exact quaternion exponential
    (:meth:`Quaternion.from_axis_angle`), rather than stepping the raw
    quaternion through R^4 and renormalizing afterward the way
    :class:`RK4Integrator` does.

    This guarantees an exact unit quaternion by construction, and is exact
    for single-axis rotation of any time-varying rate (any axis, magnitude
    varying smoothly enough for Simpson's rule to capture it). It does not
    fully correct for "coning" error - the genuine non-commutativity of
    rotations about *different* axes within the same step - so it's an
    approximation, not a rigorous Munthe-Kaas dexpinv integrator, but a
    structurally different one from :class:`RK4Integrator` that composes
    via a true rotation exponential rather than a linear ODE step in the
    ambient R^4 space, which is expected to do better on multi-axis
    compound rotation.
    """

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        dt = t1 - t0
        tm = 0.5 * (t0 + t1)

        rotation_vector = (dt / 6) * (omega(t0) + 4 * omega(tm) + omega(t1))
        angle = np.linalg.norm(rotation_vector)

        result = q0 * Quaternion.from_axis_angle(rotation_vector, angle)
        return result * (1.0 / abs(result))


class MagnusIntegrator(AttitudeIntegrator):
    """
    2nd-order Magnus expansion, using the step's two endpoint samples
    ``a = omega(t0)``, ``b = omega(t1)``. The Magnus series for
    ``dq/dt = 0.5*Omega(w(t))*q`` gives a rotation vector
    ``Theta = integral(w dt) - 0.5 * double_integral([w(s), w(r)])``, where
    ``[.,.]`` is the so(3) commutator (a cross product under the vector
    representation). Assuming ``w(t)`` varies linearly between the two
    endpoints - i.e. consistent with what :class:`deadrec.interpolation.
    TwoPointLinearInterpolator` actually supplies - both integrals have a
    closed form, giving ``Theta = dt/2*(a+b) + dt**2/12*(a cross b))``
    (the sign of the cross term confirmed empirically against a
    fine-grained reference, not just derived by hand).

    Unlike :class:`MuntheKaasIntegrator`, which composes a bare Simpson's-
    rule quadrature of ``w(t)`` onto ``q0`` via one exponential with no
    correction for the non-commutativity between rotations about
    *different* axes within the step ("coning" error), this adds the
    leading commutator correction term - and is measurably more accurate
    than :class:`MuntheKaasIntegrator` whenever the rate's *direction*
    genuinely changes within a step (its cross term is exactly zero, and
    it degenerates to :class:`MuntheKaasIntegrator`'s bare quadrature,
    when the endpoint vectors are parallel - including the constant-rate
    case).

    This is *not* an exact solution even when ``w(t)`` is genuinely linear
    over the step - the Magnus series has further, uncomputed commutator
    terms in general - only 2nd-order accurate (local truncation error
    ``O(dt**3)``), same as the classical two-sample coning-compensation
    formulas in the strapdown-INS literature this is closely related to
    (see the dedicated closed-form coning-algorithm integrator for that
    more specialized approach).
    """

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        dt = t1 - t0
        a = omega(t0)
        b = omega(t1)

        rotation_vector = dt / 2 * (a + b) + (dt**2 / 12) * np.cross(a, b)
        angle = np.linalg.norm(rotation_vector)

        result = q0 * Quaternion.from_axis_angle(rotation_vector, angle)
        return result * (1.0 / abs(result))


class ConingIntegrator(AttitudeIntegrator):
    """
    Coning-corrected integrator using all three of the step's samples -
    ``a = omega(t0)``, ``m = omega(tm)``, ``b = omega(t1)`` - for the
    commutator correction, rather than just the two endpoints
    :class:`MagnusIntegrator` uses. Fitting the unique quadratic through
    all three points (the same quadratic :class:`MuntheKaasIntegrator`'s
    Simpson's-rule integral term is already implicitly based on) is a
    strictly better model of ``w(t)`` than a straight line between the
    endpoints, so the Magnus commutator correction computed from it is
    more accurate too - the coning-error class this is named for is
    exactly the error from modelling a rotating rate vector's axis too
    coarsely, and a quadratic captures curvature a line can't.

    The rotation vector is the quadratic's exact integral (Simpson's
    rule, matching :class:`MuntheKaasIntegrator`) plus the quadratic's
    commutator-correction term:

    ``Theta = dt/6*(a+4m+b) + dt^2*(1/15*(a x m) + 1/60*(a x b) + 1/15*(m x b))``

    composed onto ``q0`` via one exact exponential, same pattern as
    :class:`MuntheKaasIntegrator`/:class:`MagnusIntegrator`.

    The coefficients above were derived by numerically fitting against a
    fine-grained numerical double integral for the quadratic model (not
    hand-derived symbolically, and not reproduced from a specific
    numbered literature coefficient table - the general technique this
    belongs to is the family of Savage-style N-sample strapdown-INS
    coning algorithms / higher-order Magnus expansions), then confirmed
    two ways: the fit's residual against the numerical reference is at
    machine precision, and, as a strong independent check, setting
    ``m = (a+b)/2`` (degenerating the quadratic back to a line) reduces
    this formula *exactly* to :class:`MagnusIntegrator`'s
    ``dt^2/12*(a x b)`` term - not merely approximately, confirmed to
    machine precision.
    """

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        dt = t1 - t0
        tm = 0.5 * (t0 + t1)
        a = omega(t0)
        m = omega(tm)
        b = omega(t1)

        integral_term = dt / 6 * (a + 4 * m + b)
        commutator_correction = dt**2 * (
            (1 / 15) * np.cross(a, m) + (1 / 60) * np.cross(a, b) + (1 / 15) * np.cross(m, b)
        )
        rotation_vector = integral_term + commutator_correction
        angle = np.linalg.norm(rotation_vector)

        result = q0 * Quaternion.from_axis_angle(rotation_vector, angle)
        return result * (1.0 / abs(result))


class AdamsBashforth2Integrator(AttitudeIntegrator):
    """
    Multistep integrator: reuses the angular-rate derivative from the
    *previous* step instead of spending an extra ``omega()`` evaluation on
    it within the current one. Every other integrator in this module is
    single-step - it sees only the current ``[t0, t1]`` in isolation, as
    :meth:`AttitudeIntegrator.integrate`'s signature implies. This one is
    stateful, which needs its own design note since the interface itself
    doesn't change to accommodate it:

    **Design decision**: :meth:`integrate` keeps the exact same signature
    as every other integrator - no interface change, nothing else needs to
    implement anything new. State (the previous step's end time, end
    attitude, and start-of-step derivative) lives on the instance itself,
    written after every call. It is only ever *trusted* on the next call
    if that call's ``t0``/``q0`` exactly match the stored end time/
    attitude - i.e. this call is genuinely the immediate next step of the
    same sequence, not an unrelated or out-of-order one. If they don't
    match (first call ever, a fresh unrelated sequence sharing this
    instance, or a call made out of strict order), it transparently falls
    back to a single-step (forward-Euler) estimate instead of trusting
    stale or unrelated history - safe by construction, never silently
    wrong, just without the multistep speedup on that particular call.
    :meth:`reset` clears the stored history explicitly (e.g. before
    reusing one instance for a second, independent run).

    This matters in practice: :class:`deadrec.dead_reckoning.DeadReckoner`
    (and :class:`deadrec.ekf.GravityCorrectedEKF`, which doesn't override
    its step logic) call the integrator exactly once per sample, in strict
    order, each call's ``q0`` literally being the previous call's result -
    a genuine sequential chain, so the history path activates correctly
    there. :class:`deadrec.ekf.WindowedGravityCorrectedEKF`, by contrast,
    evaluates several candidate steps from the *same* fixed starting
    attitude while probing different look-ahead windows - not a real
    chain - so the ``q0`` check almost never matches there and this
    integrator quietly (and correctly) just falls back every time, with no
    reckoner-level changes needed to stay correct, at the cost of not
    benefiting from the speedup in that specific reckoner.

    **The maths.** For ``dq/dt = f(t, q) = 0.5*Omega(w(t))*q``, given the
    derivative at the start of this step, ``f0 = f(t0, q0)``, and the
    derivative from the start of the *previous* step, ``f_prev``, over its
    own duration ``h_prev``, the classical (fixed-step) 2nd-order
    Adams-Bashforth update is ``q1 = q0 + h*(1.5*f0 - 0.5*f_prev)`` for
    step size ``h``. That assumes equal step sizes, which real IMU sample
    timing rarely guarantees exactly, so this uses the variable-step
    generalization instead - linearly extrapolating the two known
    derivative points and integrating that extrapolation over the new
    step:

    ``Theta = h*(f0*(1 + h/(2*h_prev)) - f_prev*(h/(2*h_prev)))``

    which reduces *exactly* to the textbook ``1.5*f0 - 0.5*f_prev`` formula
    when ``h == h_prev`` (confirmed to machine precision as a permanent
    regression test, not just during derivation). Composed the same way
    :class:`RK4Integrator` composes its stages: ``q1 = q0 + Theta``,
    renormalized - not via an exponential, since this integrates in the
    ambient quaternion space rather than building a rotation vector.

    At equal per-call cost (one ``omega()`` evaluation, same as
    :class:`EulerIntegrator`), this is markedly more accurate once history
    has built up over a few small, uniformly-spaced steps - but for a
    single large step relative to the motion's own timescale, the
    extrapolation can actually overshoot and do *worse* than plain Euler,
    since multistep predictor accuracy is an asymptotic (``h -> 0``)
    guarantee, not a per-step one regardless of step size. Both behaviours
    are confirmed empirically, not just asserted.
    """

    def __init__(self) -> None:
        self._history: tuple[float, np.ndarray, np.ndarray, float] | None = None

    def reset(self) -> None:
        self._history = None

    def integrate(
        self, q0: Quaternion, omega: Callable[[float], np.ndarray], t0: float, t1: float
    ) -> Quaternion:
        q0_vec = np.array([q0.w, q0.x, q0.y, q0.z])
        f0 = 0.5 * omega_matrix(omega(t0)) @ q0_vec
        h = t1 - t0

        has_continuous_history = self._history is not None and (
            abs(self._history[0] - t0) < 1e-9 and np.allclose(self._history[1], q0_vec, atol=1e-9)
        )

        if has_continuous_history:
            _, _, f_prev, h_prev = self._history
            theta = h * (f0 * (1 + h / (2 * h_prev)) - f_prev * (h / (2 * h_prev)))
        else:
            theta = h * f0

        q1_vec = q0_vec + theta
        result = Quaternion(*q1_vec)
        result = result * (1.0 / abs(result))

        self._history = (t1, np.array([result.w, result.x, result.y, result.z]), f0, h)

        return result
