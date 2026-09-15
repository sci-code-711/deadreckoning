"""Trajectory reconstruction by direct integration of IMU measurements."""

import numpy as np

from .kinematics import rk4_attitude_step
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

    """

    def __init__(
        self,
        initial_attitude: Quaternion,
        gravity_magnitude: float,
        *,
        gravity_direction=(0, 0, 1),
    ):
        self.attitude = initial_attitude
        self.gravity_magnitude = gravity_magnitude
        self.gravity_direction = gravity_direction
        self._prev_sample: ImuSample | None = None
        self._prev_state: TrajectoryState | None = None

    def step(self, sample: ImuSample) -> TrajectoryState:
        """
        Process one IMU sample and advance the reconstructed trajectory.

        The first call seeds the trajectory at zero velocity and
        position, using ``initial_attitude`` rather than propagating it
        (there is no previous sample to integrate gyroscope readings
        against). Every subsequent call propagates attitude with RK4 and
        integrates velocity/position with the trapezoidal rule.

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
                velocity=np.zeros(3),
                position=np.zeros(3),
                euler=self.attitude.to_euler_angles(),
            )
        else:
            prev = self._prev_state
            dt = sample.t - self._prev_sample.t

            self.attitude = rk4_attitude_step(
                self.attitude, self._prev_sample.gyro, sample.gyro, dt
            )
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

        self._prev_sample = sample
        self._prev_state = state

        return state

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
