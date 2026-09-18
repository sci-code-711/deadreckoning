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
