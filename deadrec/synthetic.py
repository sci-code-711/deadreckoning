"""Synthetic IMU trajectories with exact, closed-form ground truth.

A :class:`SyntheticTrajectory` is built from a chain of :class:`MotionSegment`
objects, each holding a constant nav-frame linear acceleration and a constant
body-frame angular velocity for a fixed duration. Because both quantities are
piecewise-constant, attitude, velocity and position all have exact closed-form
solutions in ``t`` - unlike a numerically-integrated reference, this ground
truth is independent of sample rate and introduces no discretisation error of
its own, so it can be used to benchmark reconstruction algorithms
(:class:`deadrec.dead_reckoning.DeadReckoner` and its EKF subclasses) at
whatever sample rate is being tested.
"""

import numpy as np

from .quaternion import Quaternion
from .samples import ImuSample, TrajectoryState


def _as_vec3(value, name: str) -> np.ndarray:
    vec = np.asarray(value, dtype=float)
    if vec.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {vec.shape}")
    return vec


class MotionSegment:
    """
    One leg of a synthetic trajectory: a constant nav-frame linear
    acceleration and a constant body-frame angular velocity, held for
    ``duration`` seconds.

    Args:
        * duration {``float``} -- Length of this segment, in seconds. Must
          be positive.
        * accel_nav {``array-like``} -- Constant linear acceleration in the
          navigation frame, with gravity excluded (matching
          :attr:`deadrec.samples.TrajectoryState.accel_nav`). Defaults to
          ``(0, 0, 0)``.
        * angular_velocity_body {``array-like``} -- Constant angular
          velocity in the body frame, in degrees/s (matching
          :attr:`deadrec.samples.ImuSample.gyro`). Defaults to ``(0, 0, 0)``.

    """

    def __init__(
        self, duration: float, accel_nav=(0.0, 0.0, 0.0), angular_velocity_body=(0.0, 0.0, 0.0)
    ):
        if duration <= 0:
            raise ValueError(f"duration must be positive, got {duration}")

        self.duration = float(duration)
        self.accel_nav = _as_vec3(accel_nav, "accel_nav")
        self.angular_velocity_body = _as_vec3(angular_velocity_body, "angular_velocity_body")


def _advance(
    attitude: Quaternion,
    velocity: np.ndarray,
    position: np.ndarray,
    segment: MotionSegment,
    dt: float,
):
    """
    Closed-form propagation of attitude/velocity/position over ``dt``
    seconds of ``segment``'s constant acceleration/angular velocity.

    """
    rate_rad = np.radians(segment.angular_velocity_body)
    rate = np.linalg.norm(rate_rad)

    if rate > 0:
        delta = Quaternion.from_axis_angle(rate_rad / rate, rate * dt)
        attitude = attitude * delta
        attitude = attitude * (1.0 / abs(attitude))

    new_velocity = velocity + segment.accel_nav * dt
    new_position = position + velocity * dt + 0.5 * segment.accel_nav * dt**2

    return attitude, new_velocity, new_position


def _nav_accel_to_body_frame(
    accel_nav, attitude: Quaternion, gravity_magnitude: float, gravity_direction
) -> np.ndarray:
    """
    Inverse of :func:`deadrec.dead_reckoning.accel_to_nav_frame`: given a
    true nav-frame linear acceleration (gravity excluded), find the raw
    body-frame accelerometer reading that would produce it once rotated
    into the navigation frame and gravity-corrected.

    """
    nav_vec = np.asarray(accel_nav, dtype=float) + gravity_magnitude * np.asarray(
        gravity_direction, dtype=float
    )
    nav_quat = Quaternion(0, *nav_vec)
    rotated = attitude.conjugate() * nav_quat * attitude

    return np.array([rotated.x, rotated.y, rotated.z])


class SyntheticTrajectory:
    """
    A synthetic trajectory with exact, closed-form ground truth, built from
    a chain of :class:`MotionSegment` objects.

    Args:
        * segments {``list[MotionSegment]``} -- The segments making up this
          trajectory, in order. Must be non-empty.
        * initial_attitude {``Quaternion``} -- Attitude at ``t=0``. Defaults
          to the identity quaternion.
        * initial_velocity {``array-like``} -- Velocity at ``t=0``. Defaults
          to ``(0, 0, 0)``.
        * initial_position {``array-like``} -- Position at ``t=0``. Defaults
          to ``(0, 0, 0)``.
        * gravity_magnitude {``float``} -- Magnitude of gravitational
          acceleration, matching :class:`deadrec.dead_reckoning.DeadReckoner`.
          Defaults to ``9.80665``.
        * gravity_direction {``array-like``} -- Unit vector giving the
          direction of gravity in the navigation frame. Defaults to
          ``(0, 0, 1)``.

    """

    def __init__(
        self,
        segments: list[MotionSegment],
        *,
        initial_attitude: Quaternion = None,
        initial_velocity=(0.0, 0.0, 0.0),
        initial_position=(0.0, 0.0, 0.0),
        gravity_magnitude: float = 9.80665,
        gravity_direction=(0.0, 0.0, 1.0),
    ):
        if not segments:
            raise ValueError("segments must be non-empty")

        self.segments = list(segments)
        self.gravity_magnitude = gravity_magnitude
        self.gravity_direction = _as_vec3(gravity_direction, "gravity_direction")

        if initial_attitude is None:
            initial_attitude = Quaternion(1, 0, 0, 0)

        self._boundaries = np.concatenate(([0.0], np.cumsum([s.duration for s in self.segments])))
        self._segment_starts = self._compute_segment_starts(
            initial_attitude,
            _as_vec3(initial_velocity, "initial_velocity"),
            _as_vec3(initial_position, "initial_position"),
        )

    @property
    def duration(self) -> float:
        """The trajectory's total duration, in seconds."""
        return float(self._boundaries[-1])

    def _compute_segment_starts(self, attitude, velocity, position):
        starts = []
        for segment in self.segments:
            starts.append((attitude, velocity, position))
            attitude, velocity, position = _advance(
                attitude, velocity, position, segment, segment.duration
            )

        return starts

    def _evaluate(self, t: float):
        if t < -1e-9 or t > self.duration + 1e-9:
            raise ValueError(f"t={t} is outside this trajectory's duration [0, {self.duration}]")

        idx = int(np.searchsorted(self._boundaries, t, side="right")) - 1
        idx = min(max(idx, 0), len(self.segments) - 1)

        segment = self.segments[idx]
        q0, v0, p0 = self._segment_starts[idx]
        elapsed = min(max(t - self._boundaries[idx], 0.0), segment.duration)

        attitude, velocity, position = _advance(q0, v0, p0, segment, elapsed)

        return segment, attitude, velocity, position

    def state_at(self, t: float) -> TrajectoryState:
        """
        The exact ground-truth :class:`TrajectoryState` at time ``t``.

        Args:
            * t {``float``} -- Time, in seconds, within ``[0, self.duration]``.

        Returns:
            * {``TrajectoryState``}

        """
        segment, attitude, velocity, position = self._evaluate(t)

        return TrajectoryState(
            t=t,
            attitude=attitude,
            accel_nav=segment.accel_nav,
            velocity=velocity,
            position=position,
            euler=attitude.to_euler_angles(),
        )

    def imu_sample_at(self, t: float) -> ImuSample:
        """
        The exact, noiseless :class:`ImuSample` reading at time ``t``.

        Args:
            * t {``float``} -- Time, in seconds, within ``[0, self.duration]``.

        Returns:
            * {``ImuSample``}

        """
        segment, attitude, _velocity, _position = self._evaluate(t)
        accel_body = _nav_accel_to_body_frame(
            segment.accel_nav, attitude, self.gravity_magnitude, self.gravity_direction
        )

        return ImuSample(t=t, accel=accel_body, gyro=segment.angular_velocity_body)

    def sample(self, sample_times) -> tuple[list[TrajectoryState], list[ImuSample]]:
        """
        Evaluate this trajectory's ground truth and IMU readings at
        ``sample_times``.

        Args:
            * sample_times {``array-like``} -- Times, in seconds, to sample
              at. Each must lie within ``[0, self.duration]``.

        Returns:
            * {``tuple``} -- ``(ground_truth, samples)``: the
              :class:`TrajectoryState` and :class:`ImuSample` at each of
              ``sample_times``, in order.

        """
        sample_times = [float(t) for t in sample_times]
        ground_truth = [self.state_at(t) for t in sample_times]
        samples = [self.imu_sample_at(t) for t in sample_times]

        return ground_truth, samples

    def sample_at_rate(self, hz: float) -> tuple[list[TrajectoryState], list[ImuSample]]:
        """
        Evaluate this trajectory's ground truth and IMU readings at an even
        sample rate, from ``t=0`` up to (and including) ``self.duration``.

        Args:
            * hz {``float``} -- Sampling rate, in Hz. Must be positive.

        Returns:
            * {``tuple``} -- As :meth:`sample`.

        """
        if hz <= 0:
            raise ValueError(f"hz must be positive, got {hz}")

        n_steps = int(np.floor(self.duration * hz + 1e-9))
        sample_times = np.arange(n_steps + 1) / hz

        return self.sample(sample_times)
