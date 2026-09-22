"""Attitude keyframe spline: SQUAD (spherical cubic quaternion
interpolation) over a sequence of orientation keyframes.

Companion to the position-spline building blocks added alongside it for
#41's Phase 3 groundwork - the same idea applied to attitude: rather than
integrating forward from an authored rate (as
:class:`deadrec.synthetic.SyntheticTrajectory` does), the orientation
curve itself is authored directly as a sequence of keyframes and
interpolated, with the angular rate derived from the resulting curve
rather than the other way around.
"""

import numpy as np

from .quaternion import Quaternion

_ANGULAR_VELOCITY_STEP = 1e-6


def _log(q: Quaternion) -> np.ndarray:
    """
    Quaternion logarithm of a unit quaternion, as the pure imaginary
    3-vector ``theta * axis`` (half the rotation angle, matching
    :meth:`Quaternion.from_axis_angle`'s half-angle convention), such that
    :func:`_exp` inverts it.

    """
    v = np.array([q.x, q.y, q.z])
    v_norm = np.linalg.norm(v)

    if v_norm < 1e-12:
        return np.zeros(3)

    theta = np.arctan2(v_norm, q.w)

    return (theta / v_norm) * v


def _exp(v: np.ndarray) -> Quaternion:
    """Quaternion exponential of a pure imaginary 3-vector - the inverse of :func:`_log`."""
    theta = np.linalg.norm(v)

    if theta < 1e-12:
        return Quaternion(1.0, 0.0, 0.0, 0.0)

    return Quaternion(np.cos(theta), *(np.sin(theta) / theta * v))


def _squad_control_point(q_prev: Quaternion, q: Quaternion, q_next: Quaternion) -> Quaternion:
    """
    The intermediate SQUAD control quaternion at an interior keyframe
    ``q``, following Shoemake's construction: a rotation that bisects the
    "incoming" and "outgoing" tangents at ``q``, so the curve turns
    smoothly through it rather than kinking as plain SLERP would.

    """
    q_inv = q.conjugate()
    tangent = _log(q_inv * q_prev) + _log(q_inv * q_next)

    return q * _exp(-0.25 * tangent)


def _squad(q0: Quaternion, q1: Quaternion, s0: Quaternion, s1: Quaternion, h: float) -> Quaternion:
    """
    SQUAD interpolation between keyframes ``q0``/``q1`` with control
    quaternions ``s0``/``s1``, at fraction ``h`` of the segment.

    """
    return Quaternion.slerp(
        Quaternion.slerp(q0, q1, h), Quaternion.slerp(s0, s1, h), 2 * h * (1 - h)
    )


class AttitudeSpline:
    """
    A smooth orientation curve through a sequence of keyframes, using
    SQUAD (spherical cubic quaternion interpolation - nested SLERPs
    through per-keyframe control quaternions) rather than plain SLERP, so
    the curve's angular rate is continuous across keyframes instead of
    changing abruptly at each one.

    Args:
        * keyframes {``Sequence[tuple[float, Quaternion]]``} -- The
          ``(t, attitude)`` keyframes to interpolate through, in strictly
          increasing order of ``t``. Must contain at least two.

    """

    def __init__(self, keyframes):
        keyframes = list(keyframes)
        if len(keyframes) < 2:
            raise ValueError(f"keyframes must contain at least two entries, got {len(keyframes)}")

        times = [t for t, _ in keyframes]
        if any(t2 <= t1 for t1, t2 in zip(times, times[1:])):
            raise ValueError("keyframe times must be strictly increasing")

        self._times = np.array(times, dtype=float)
        self._quats = [q for _, q in keyframes]
        self._controls = self._build_controls()

    @property
    def t0(self) -> float:
        """This spline's first keyframe time, in seconds."""
        return float(self._times[0])

    @property
    def t1(self) -> float:
        """This spline's last keyframe time, in seconds."""
        return float(self._times[-1])

    def _build_controls(self) -> list:
        n = len(self._quats)
        interior = [
            _squad_control_point(self._quats[i - 1], self._quats[i], self._quats[i + 1])
            for i in range(1, n - 1)
        ]

        return [self._quats[0], *interior, self._quats[-1]]

    def _segment(self, t: float) -> tuple:
        if t < self.t0 - 1e-9 or t > self.t1 + 1e-9:
            raise ValueError(f"t={t} is outside this spline's range [{self.t0}, {self.t1}]")

        idx = int(np.searchsorted(self._times, t, side="right")) - 1
        idx = min(max(idx, 0), len(self._quats) - 2)

        span = self._times[idx + 1] - self._times[idx]
        h = (t - self._times[idx]) / span
        h = min(max(h, 0.0), 1.0)

        return idx, h

    def attitude(self, t: float) -> Quaternion:
        """
        The interpolated attitude at time ``t``.

        Args:
            * t {``float``} -- Time, in seconds, within
              ``[self.t0, self.t1]``.

        Returns:
            * {``Quaternion``}

        """
        idx, h = self._segment(t)

        return _squad(
            self._quats[idx],
            self._quats[idx + 1],
            self._controls[idx],
            self._controls[idx + 1],
            h,
        )

    def angular_velocity_body(self, t: float) -> np.ndarray:
        """
        The body-frame angular velocity at time ``t``, in degrees/s
        (matching :attr:`deadrec.samples.ImuSample.gyro`), estimated by a
        tiny-step (``1e-6`` s, fixed - decoupled from any sample rate a
        caller might request) central difference of :meth:`attitude`,
        rather than a hand-derived closed-form SQUAD-derivative formula.

        Args:
            * t {``float``} -- Time, in seconds, within
              ``[self.t0, self.t1]``.

        Returns:
            * {``np.ndarray``} -- The ``(x, y, z)`` body-frame angular
              velocity, in degrees/s.

        """
        t_minus = max(t - _ANGULAR_VELOCITY_STEP, self.t0)
        t_plus = min(t + _ANGULAR_VELOCITY_STEP, self.t1)
        dt = t_plus - t_minus

        if dt <= 0:
            return np.zeros(3)

        q_minus = self.attitude(t_minus)
        q_plus = self.attitude(t_plus)

        delta = q_minus.conjugate() * q_plus
        delta = delta * (1.0 / abs(delta))

        v = np.array([delta.x, delta.y, delta.z])
        v_norm = np.linalg.norm(v)

        if v_norm < 1e-12:
            return np.zeros(3)

        phi = np.arctan2(v_norm, delta.w)
        rate_rad = (v / v_norm) * (2 * phi / dt)

        return np.degrees(rate_rad)
