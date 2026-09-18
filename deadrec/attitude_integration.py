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
