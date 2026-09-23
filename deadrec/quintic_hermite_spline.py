"""Quintic Hermite waypoint-interpolating position curve.

Part of #41's Phase 3 groundwork - sibling of the (separately ticketed)
natural cubic spline. Where a natural cubic spline solves one *global*
linear system across every waypoint to find accelerations implicitly,
this curve takes acceleration as a direct, optional per-waypoint input
(defaulting to zero), so building one segment never depends on any other
segment or waypoint beyond its own two endpoints plus one neighbour each
side for a tangent estimate.
"""

import numpy as np

from .curves import PolynomialCurve

_DEGREE = 5


def _derivative_row(t: float, order: int) -> np.ndarray:
    """
    The row of coefficients such that ``row @ c`` gives the ``order``-th
    derivative at ``t`` of the degree-``_DEGREE`` polynomial with monomial
    coefficients ``c`` (``c[k]`` multiplying ``t**k``).

    """
    row = np.zeros(_DEGREE + 1)
    for k in range(order, _DEGREE + 1):
        falling_factorial = 1
        for i in range(order):
            falling_factorial *= k - i
        row[k] = falling_factorial * t ** (k - order)

    return row


def _segment_curve(
    t0: float,
    t1: float,
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
) -> PolynomialCurve:
    """
    The unique degree-5 :class:`PolynomialCurve` matching position,
    velocity and acceleration exactly at both ``t0`` and ``t1`` - found by
    solving the small, local 6x6 linear system directly for the monomial
    coefficients (equivalent to the standard quintic-Hermite basis-function
    formulation, just computed generically rather than from hand-transcribed
    basis polynomials).

    """
    matrix = np.array(
        [
            _derivative_row(t0, 0),
            _derivative_row(t0, 1),
            _derivative_row(t0, 2),
            _derivative_row(t1, 0),
            _derivative_row(t1, 1),
            _derivative_row(t1, 2),
        ]
    )
    boundary_values = np.array([p0, v0, a0, p1, v1, a1])

    return PolynomialCurve(np.linalg.solve(matrix, boundary_values))


def _parse_waypoint(waypoint) -> tuple:
    if len(waypoint) == 2:
        t, p = waypoint
        a = np.zeros(3)
    elif len(waypoint) == 3:
        t, p, a = waypoint
        a = np.zeros(3) if a is None else np.asarray(a, dtype=float)
    else:
        raise ValueError(
            f"each waypoint must be a (t, position) or (t, position, acceleration) "
            f"tuple, got {waypoint!r}"
        )

    return float(t), np.asarray(p, dtype=float), a


class QuinticHermitePositionSpline:
    """
    A C2-smooth position curve through a sequence of waypoints, using one
    degree-5 Hermite polynomial per segment.

    Each waypoint is ``(t, position)`` or ``(t, position, acceleration)`` -
    acceleration defaults to zero when omitted, so a plain
    ``[(t, position), ...]`` list (matching the natural-cubic-spline's
    waypoint format) works directly. Velocity at each waypoint is not an
    input - it's estimated via a Catmull-Rom-style finite-difference
    secant through neighbouring waypoints (the same convention
    :class:`deadrec.interpolation.CentredCubicHermiteInterpolator` already
    uses for angular rate), falling back to the one-sided secant through
    the single neighbour available at the first/last waypoint.

    Args:
        * waypoints {``Sequence``} -- The waypoints to interpolate
          through, in strictly increasing order of ``t``. Must contain at
          least two.

    """

    def __init__(self, waypoints):
        parsed = [_parse_waypoint(w) for w in waypoints]
        if len(parsed) < 2:
            raise ValueError(f"waypoints must contain at least two entries, got {len(parsed)}")

        times = [t for t, _, _ in parsed]
        if any(t2 <= t1 for t1, t2 in zip(times, times[1:])):
            raise ValueError("waypoint times must be strictly increasing")

        self._times = np.array(times, dtype=float)
        self._positions = [p for _, p, _ in parsed]
        self._accelerations = [a for _, _, a in parsed]
        self._velocities = self._estimate_velocities()
        self._segments = self._build_segments()

    @property
    def t0(self) -> float:
        """This spline's first waypoint time, in seconds."""
        return float(self._times[0])

    @property
    def t1(self) -> float:
        """This spline's last waypoint time, in seconds."""
        return float(self._times[-1])

    def _estimate_velocities(self) -> list:
        n = len(self._positions)
        velocities = []
        for i in range(n):
            if i == 0:
                left, right = 0, 1
            elif i == n - 1:
                left, right = n - 2, n - 1
            else:
                left, right = i - 1, i + 1

            span = self._times[right] - self._times[left]
            velocities.append((self._positions[right] - self._positions[left]) / span)

        return velocities

    def _build_segments(self) -> list:
        return [
            _segment_curve(
                self._times[i],
                self._times[i + 1],
                self._positions[i],
                self._velocities[i],
                self._accelerations[i],
                self._positions[i + 1],
                self._velocities[i + 1],
                self._accelerations[i + 1],
            )
            for i in range(len(self._times) - 1)
        ]

    def _segment_at(self, t: float) -> PolynomialCurve:
        if t < self.t0 - 1e-9 or t > self.t1 + 1e-9:
            raise ValueError(f"t={t} is outside this spline's range [{self.t0}, {self.t1}]")

        idx = int(np.searchsorted(self._times, t, side="right")) - 1
        idx = min(max(idx, 0), len(self._segments) - 1)

        return self._segments[idx]

    def value(self, t: float) -> np.ndarray:
        """Position at ``t``, as a (3,) array."""
        return self._segment_at(t).value(t)

    def velocity(self, t: float) -> np.ndarray:
        """Velocity at ``t``, as a (3,) array."""
        return self._segment_at(t).velocity(t)

    def acceleration(self, t: float) -> np.ndarray:
        """Acceleration at ``t``, as a (3,) array."""
        return self._segment_at(t).acceleration(t)
