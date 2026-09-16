"""Trajectory reconstruction by direct integration of IMU measurements."""

from collections import deque

import numpy as np

from .attitude_integration import AttitudeIntegrator, RK4Integrator
from .interpolation import AngularRateInterpolator, TwoPointLinearInterpolator
from .quaternion import Quaternion
from .samples import ImuSample, TrajectoryState


def accel_to_nav_frame(
    accel, attitude: Quaternion, gravity_magnitude: float, gravity_direction=(0, 0, 1)
) -> np.ndarray:
    """
    Rotate a body-frame acceleration reading into the navigation frame and
    remove the gravitational component.

    Args:
        * accel {``array-like``} -- Body-frame acceleration reading.
        * attitude {``Quaternion``} -- Current attitude (body-to-navigation
          rotation).
        * gravity_magnitude {``float``} -- Magnitude of gravitational
          acceleration to remove, e.g. from :func:`estimate_gravity_magnitude`.
        * gravity_direction {``array-like``} -- Unit vector giving the
          direction of gravity in the navigation frame. Defaults to
          ``(0, 0, 1)``.

    Returns:
        * {``np.ndarray``} -- (3,) gravity-removed acceleration in the
          navigation frame.

    """
    accel_quat = Quaternion(0, *accel)
    rotated = attitude * accel_quat * attitude.conjugate()
    nav = np.array([rotated.x, rotated.y, rotated.z])

    return nav - gravity_magnitude * np.asarray(gravity_direction, dtype=float)


class DeadReckoner:
    """
    Reconstructs a trajectory from a stream of IMU samples by directly
    integrating gyroscope readings (via 4th order Runge-Kutta) to track
    attitude, then integrating gravity-removed, navigation-frame
    acceleration to velocity and position.

    Args:
        * initial_attitude {``Quaternion``} -- Attitude at the first sample,
          e.g. from :func:`deadrec.attitude.initial_attitude_from_gravity`.
        * gravity_magnitude {``float``} -- Magnitude of gravitational
          acceleration, e.g. from
          :func:`deadrec.attitude.estimate_gravity_magnitude`.
        * gravity_direction {``array-like``} -- Unit vector giving the
          direction of gravity in the navigation frame. Defaults to
          ``(0, 0, 1)``.
        * initial_velocity {``array-like``} -- Velocity at the first sample.
          Defaults to ``(0, 0, 0)`` - only correct if the system is known to
          be at rest there; pass the real velocity when resuming a
          trajectory (e.g. from a previous run's final state) instead.
        * initial_position {``array-like``} -- Position at the first sample.
          Defaults to ``(0, 0, 0)``, i.e. treating the first sample as the
          origin; pass the real position when it's known.
        * interpolator {``AngularRateInterpolator``} -- Strategy used to
          turn gyro readings into a continuous angular-rate function for
          each step. Defaults to :class:`~deadrec.interpolation.TwoPointLinearInterpolator`,
          reproducing this class's original hardcoded behavior. Must be
          causal (see :attr:`~deadrec.interpolation.AngularRateInterpolator.causal`) -
          use :class:`~deadrec.ekf.WindowedGravityCorrectedEKF` for a
          non-causal one.
        * integrator {``AttitudeIntegrator``} -- Strategy used to integrate
          attitude across each step given the interpolator's angular-rate
          function. Defaults to :class:`~deadrec.attitude_integration.RK4Integrator`.

    """

    def __init__(
        self,
        initial_attitude: Quaternion,
        gravity_magnitude: float,
        *,
        gravity_direction=(0, 0, 1),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_position=(0.0, 0.0, 0.0),
        interpolator: AngularRateInterpolator = TwoPointLinearInterpolator(),
        integrator: AttitudeIntegrator = RK4Integrator(),
    ):
        self.attitude = initial_attitude
        self.gravity_magnitude = gravity_magnitude
        self.gravity_direction = gravity_direction
        self.initial_velocity = np.asarray(initial_velocity, dtype=float)
        self.initial_position = np.asarray(initial_position, dtype=float)
        self.interpolator = interpolator
        self.integrator = integrator
        self._check_interpolator_compatibility(self.interpolator)
        self._history: deque[ImuSample] = deque(maxlen=self.interpolator.context_before + 1)
        self._prev_state: TrajectoryState | None = None

    def _check_interpolator_compatibility(self, interpolator: AngularRateInterpolator) -> None:
        """
        Reject an interpolator that needs look-ahead samples this streaming
        reckoner can't provide.

        Subclasses that always hold the full sample sequence up front
        (i.e. :class:`~deadrec.ekf.WindowedGravityCorrectedEKF`) override
        this to a no-op.

        Args:
            * interpolator {``AngularRateInterpolator``} -- The
              interpolator to check.

        """
        if not interpolator.causal:
            raise ValueError(
                f"{type(interpolator).__name__} is not causal (it needs "
                f"look-ahead samples), so it can't be used with {type(self).__name__}, "
                "which processes samples one at a time as they arrive. Use "
                "WindowedGravityCorrectedEKF instead, which holds the full "
                "sample sequence up front."
            )

    def step(self, sample: ImuSample) -> TrajectoryState:
        """
        Process one IMU sample and advance the reconstructed trajectory.

        The first call seeds the trajectory at ``initial_velocity``/
        ``initial_position`` (zero by default) using ``initial_attitude``
        rather than propagating it - there is no previous sample to
        integrate gyroscope readings against. Every subsequent call
        propagates attitude with RK4 and integrates velocity/position with
        the trapezoidal rule.

        Args:
            * sample {``ImuSample``} -- The IMU reading to process.

        Returns:
            * {``TrajectoryState``} -- The reconstructed state at ``sample.t``.

        """
        if self._prev_state is None:
            accel_nav = accel_to_nav_frame(
                sample.accel, self.attitude, self.gravity_magnitude, self.gravity_direction
            )
            state = TrajectoryState(
                t=sample.t,
                attitude=self.attitude,
                accel_nav=accel_nav,
                velocity=self.initial_velocity,
                position=self.initial_position,
                euler=self.attitude.to_euler_angles(),
            )
        else:
            prev = self._prev_state
            dt = sample.t - self._history[-1].t

            self.attitude = self._predict_attitude(sample, dt)
            accel_nav = accel_to_nav_frame(
                sample.accel, self.attitude, self.gravity_magnitude, self.gravity_direction
            )

            velocity = prev.velocity + (prev.accel_nav + accel_nav) * dt / 2
            position = prev.position + prev.velocity * dt + (prev.accel_nav + accel_nav) * dt**2 / 4

            state = TrajectoryState(
                t=sample.t,
                attitude=self.attitude,
                accel_nav=accel_nav,
                velocity=velocity,
                position=position,
                euler=self.attitude.to_euler_angles(),
            )

        self._history.append(sample)
        self._prev_state = state

        return state

    def _predict_attitude(self, sample: ImuSample, dt: float) -> Quaternion:
        """
        Propagate attitude to ``sample`` using gyroscope readings.

        Subclasses that fuse in other sensors (e.g. a gravity-vector
        correction) can override this to adjust the prediction before it's
        used to rotate acceleration into the navigation frame.

        Args:
            * sample {``ImuSample``} -- The IMU reading being processed.
              ``self._history[-1]`` is the previous one.
            * dt {``float``} -- Time step since ``self._history[-1]`` (s).

        Returns:
            * {``Quaternion``} -- The predicted attitude at ``sample.t``.

        """
        window = list(self._history) + [sample]
        step_pos = len(self._history)
        omega_fn = self.interpolator.build(window, step_pos)

        return self.integrator.integrate(self.attitude, omega_fn, self._history[-1].t, sample.t)

    def run(self, samples) -> list[TrajectoryState]:
        """
        Process a sequence of IMU samples in order.

        Args:
            * samples {``Iterable[ImuSample]``} -- The samples to process,
              in time order.

        Returns:
            * {``list[TrajectoryState]``} -- The reconstructed state at
              each sample.

        """
        return [self.step(sample) for sample in samples]
