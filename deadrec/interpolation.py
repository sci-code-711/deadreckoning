"""Pluggable angular-rate interpolation strategies for attitude propagation.

An :class:`AngularRateInterpolator` turns the samples spanning one
integration step (plus, for some strategies, extra context samples before
and/or after) into a continuous ``omega(t)`` callable. An
:class:`deadrec.attitude_integration.AttitudeIntegrator` then queries that
callable at whatever times its scheme needs - the two are orthogonal, so
interpolators and integrators can be combined freely.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence

import numpy as np

from .samples import ImuSample


class AngularRateInterpolator(ABC):
    """
    Builds a continuous angular-rate function from IMU samples spanning one
    integration step, and optionally some context beyond it.

    Attributes:
        * context_before {``int``} -- Extra past samples :meth:`build`
          needs, beyond the step's own start sample. Defaults to ``0``.
        * context_after {``int``} -- Extra future samples :meth:`build`
          needs, beyond the step's own end sample. Defaults to ``0``;
          ``> 0`` means the interpolator needs samples that haven't
          happened yet relative to the step.

    """

    context_before: int = 0
    context_after: int = 0

    @property
    def needs_lookahead(self) -> bool:
        """
        Whether this interpolator needs samples that haven't happened yet
        relative to the step's end sample, and so can only be used by a
        batch reckoner that holds the full sample sequence up front, not a
        streaming reckoner that processes one sample at a time.

        Returns:
            * {``bool``} -- ``True`` iff ``context_after > 0``.

        """
        return self.context_after > 0

    @abstractmethod
    def build(self, window: Sequence[ImuSample], step_pos: int) -> Callable[[float], np.ndarray]:
        """
        Build ``omega(t)``, the interpolated body-frame angular rate in
        rad/s, for the step from ``window[step_pos - 1]`` to
        ``window[step_pos]``.

        Args:
            * window {``Sequence[ImuSample]``} -- Samples available around
              the step, including ``context_before``/``context_after``
              extra samples either side where applicable.
            * step_pos {``int``} -- Index into ``window`` of the step's end
              sample; ``window[step_pos - 1]`` is its start sample.

        Returns:
            * {``Callable[[float], np.ndarray]``} -- ``omega(t)``, in rad/s,
              defined at least over
              ``[window[step_pos - 1].t, window[step_pos].t]``.

        """


class TwoPointLinearInterpolator(AngularRateInterpolator):
    """Linearly blends the step's two endpoint gyro readings."""

    def build(self, window: Sequence[ImuSample], step_pos: int) -> Callable[[float], np.ndarray]:
        start, end = window[step_pos - 1], window[step_pos]
        w0 = np.radians(start.gyro)
        w1 = np.radians(end.gyro)
        t0, t1 = start.t, end.t
        span = t1 - t0

        def omega(t: float) -> np.ndarray:
            if span == 0:
                return w0
            frac = (t - t0) / span
            return w0 + frac * (w1 - w0)

        return omega


class ZeroOrderHoldInterpolator(AngularRateInterpolator):
    """
    Holds the step's starting gyro reading constant over the whole step.
    Causal, and the cheapest strategy available - no interpolation math at
    all.
    """

    def build(self, window: Sequence[ImuSample], step_pos: int) -> Callable[[float], np.ndarray]:
        w0 = np.radians(window[step_pos - 1].gyro)

        def omega(t: float) -> np.ndarray:
            return w0

        return omega


class CentredCubicHermiteInterpolator(AngularRateInterpolator):
    """
    Cubic Hermite interpolation through the step's two endpoint samples,
    with finite-difference ("Catmull-Rom"-style) tangents estimated from
    one extra sample of context either side. Non-causal - needs one sample
    of look-ahead beyond the step's end sample, so only usable by a batch
    reckoner that holds the full sample sequence up front (i.e.
    :class:`deadrec.ekf.WindowedGravityCorrectedEKF`).

    Near either end of a sample sequence, gracefully clips to whatever
    context is actually available rather than raising - falling back to a
    one-sided finite difference (or, with no context available at all, a
    tangent equal to the endpoint secant, making the interpolant reduce
    exactly to linear) instead of a centred one.
    """

    context_before = 1
    context_after = 1

    def build(self, window: Sequence[ImuSample], step_pos: int) -> Callable[[float], np.ndarray]:
        start, end = window[step_pos - 1], window[step_pos]
        t0, t1 = start.t, end.t
        p0 = np.radians(start.gyro)
        p1 = np.radians(end.gyro)
        span = t1 - t0
        secant = (p1 - p0) / span if span != 0 else np.zeros(3)

        before = window[step_pos - 2] if step_pos - 2 >= 0 else None
        after = window[step_pos + 1] if step_pos + 1 < len(window) else None

        if before is not None and before.t != t1:
            m0 = (p1 - np.radians(before.gyro)) / (t1 - before.t)
        else:
            m0 = secant

        if after is not None and after.t != t0:
            m1 = (np.radians(after.gyro) - p0) / (after.t - t0)
        else:
            m1 = secant

        def omega(t: float) -> np.ndarray:
            if span == 0:
                return p0
            s = (t - t0) / span
            h00 = 2 * s**3 - 3 * s**2 + 1
            h10 = s**3 - 2 * s**2 + s
            h01 = -2 * s**3 + 3 * s**2
            h11 = s**3 - s**2
            return h00 * p0 + h10 * span * m0 + h01 * p1 + h11 * span * m1

        return omega
