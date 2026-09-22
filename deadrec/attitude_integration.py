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
