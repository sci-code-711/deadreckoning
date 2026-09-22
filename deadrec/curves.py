"""Differentiable polynomial curves, the building block for the
waypoint-driven position splines (see #41).
"""

import numpy as np
from numpy.polynomial import Polynomial


class PolynomialCurve:
    """
    A 3-vector-valued polynomial in one variable ``t``, differentiable
    exactly via ``numpy.polynomial.Polynomial.deriv()``.

    This is the shared contract every position-spline segment in this
    package is built to: ``.value(t)``/``.velocity(t)``/``.acceleration(t)``,
    each returning a (3,) array. ``NaturalCubicPositionSpline`` and
    ``QuinticHermitePositionSpline`` both expose this same interface
    (duck-typed, no formal ABC - matching how ``MotionSegment``/
    ``CoordinatedTurnSegment`` in :mod:`deadrec.synthetic` already share an
    ``evaluate(v0, p0, dt)`` contract without one).

    Args:
        * coefficients {``array-like``} -- (degree + 1, 3) array; row ``k``
          holds the coefficient of ``t^k`` for each of the x/y/z axes
          (columns) - :class:`numpy.polynomial.Polynomial`'s own
          coefficient convention, one column per axis.

    """

    def __init__(self, coefficients):
        coefficients = np.asarray(coefficients, dtype=float)
        if coefficients.ndim != 2 or coefficients.shape[1] != 3:
            raise ValueError(
                f"coefficients must have shape (degree + 1, 3), got {coefficients.shape}"
            )

        self._polynomials = [Polynomial(coefficients[:, axis]) for axis in range(3)]
        self._velocity_polynomials = [p.deriv() for p in self._polynomials]
        self._acceleration_polynomials = [p.deriv() for p in self._velocity_polynomials]

    def value(self, t: float) -> np.ndarray:
        """Position at ``t``, as a (3,) array."""
        return np.array([p(t) for p in self._polynomials])

    def velocity(self, t: float) -> np.ndarray:
        """Exact first derivative at ``t``, as a (3,) array."""
        return np.array([p(t) for p in self._velocity_polynomials])

    def acceleration(self, t: float) -> np.ndarray:
        """Exact second derivative at ``t``, as a (3,) array."""
        return np.array([p(t) for p in self._acceleration_polynomials])
