"""A C² natural cubic spline through waypoints, one of two position-spline
implementations for the waypoint-driven trajectory generator (see #41).
"""

import numpy as np
from numpy.polynomial import Polynomial

from .curves import PolynomialCurve


def _as_vec3(value, name: str) -> np.ndarray:
    vec = np.asarray(value, dtype=float)
    if vec.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {vec.shape}")
    return vec


class NaturalCubicPositionSpline:
    """
    A position curve that exactly interpolates a set of waypoints with a
    "natural" cubic spline: piecewise-cubic, with position, velocity *and*
    acceleration continuous across every waypoint (C²), and zero curvature
    imposed at the first and last waypoint (the classic "natural" boundary
    condition) rather than choosing waypoint accelerations explicitly -
    see :class:`deadrec.quintic_hermite_spline.QuinticHermitePositionSpline`
    for the sibling that takes them as an explicit input instead.

    Exposes the same ``.value(t)``/``.velocity(t)``/``.acceleration(t)``
    contract as :class:`deadrec.curves.PolynomialCurve`, spanning the whole
    waypoint range.

    Args:
        * waypoints {``Sequence[tuple[float, array-like]]``} -- The
          ``(t, position)`` pairs to interpolate, in strictly increasing
          order of ``t``. Must contain at least 2 waypoints.

    """

    def __init__(self, waypoints):
        if len(waypoints) < 2:
            raise ValueError(f"need at least 2 waypoints, got {len(waypoints)}")

        times = np.array([t for t, _ in waypoints], dtype=float)
        if np.any(np.diff(times) <= 0):
            raise ValueError("waypoint times must be strictly increasing")

        positions = np.array([_as_vec3(p, "waypoint position") for _, p in waypoints])

        self._boundaries = times
        self._segments = self._build_segments(times, positions)

    def _build_segments(self, times: np.ndarray, positions: np.ndarray) -> list[PolynomialCurve]:
        n = len(times) - 1
        h = np.diff(times)

        second_derivatives = np.zeros((n + 1, 3))
        if n >= 2:
            size = n - 1
            system = np.zeros((size, size))
            rhs = np.zeros((size, 3))

            for row, i in enumerate(range(1, n)):
                if row > 0:
                    system[row, row - 1] = h[i - 1]
                system[row, row] = 2 * (h[i - 1] + h[i])
                if row < size - 1:
                    system[row, row + 1] = h[i]

                rhs[row] = 6 * (
                    (positions[i + 1] - positions[i]) / h[i]
                    - (positions[i] - positions[i - 1]) / h[i - 1]
                )

            second_derivatives[1:n] = np.linalg.solve(system, rhs)

        tau = Polynomial([0.0, 1.0])
        segments = []

        for i in range(n):
            hi = h[i]
            m_i, m_ip1 = second_derivatives[i], second_derivatives[i + 1]
            p_i, p_ip1 = positions[i], positions[i + 1]

            coefficients = np.zeros((4, 3))
            for axis in range(3):
                segment_poly = (
                    m_i[axis] / (6 * hi) * (hi - tau) ** 3
                    + m_ip1[axis] / (6 * hi) * tau**3
                    + (p_i[axis] / hi - m_i[axis] * hi / 6) * (hi - tau)
                    + (p_ip1[axis] / hi - m_ip1[axis] * hi / 6) * tau
                )
                coefficients[: len(segment_poly.coef), axis] = segment_poly.coef

            segments.append(PolynomialCurve(coefficients))

        return segments

    def _segment_index(self, t: float) -> int:
        if t < self._boundaries[0] - 1e-9 or t > self._boundaries[-1] + 1e-9:
            raise ValueError(
                f"t={t} is outside this spline's range "
                f"[{self._boundaries[0]}, {self._boundaries[-1]}]"
            )

        idx = int(np.searchsorted(self._boundaries, t, side="right")) - 1
        return min(max(idx, 0), len(self._segments) - 1)

    def value(self, t: float) -> np.ndarray:
        """Position at ``t``, as a (3,) array."""
        idx = self._segment_index(t)
        return self._segments[idx].value(t - self._boundaries[idx])

    def velocity(self, t: float) -> np.ndarray:
        """Exact first derivative at ``t``, as a (3,) array."""
        idx = self._segment_index(t)
        return self._segments[idx].velocity(t - self._boundaries[idx])

    def acceleration(self, t: float) -> np.ndarray:
        """Exact second derivative at ``t``, as a (3,) array."""
        idx = self._segment_index(t)
        return self._segments[idx].acceleration(t - self._boundaries[idx])
