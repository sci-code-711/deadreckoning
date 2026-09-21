"""Synthetic IMU trajectories with exact, closed-form ground truth.

A :class:`SyntheticTrajectory` is built from a chain of segments, each
holding a fixed duration and a closed-form description of the motion over
that duration: either a constant nav-frame linear acceleration and constant
body-frame angular velocity (:class:`MotionSegment`), or a constant-speed,
constant-turn-rate coordinated turn (:class:`CoordinatedTurnSegment`).
Because each primitive is closed-form, attitude, velocity and position all
have exact solutions in ``t`` - unlike a numerically-integrated reference,
this ground truth is independent of sample rate and introduces no
discretisation error of its own, so it can be used to benchmark
reconstruction algorithms (:class:`deadrec.dead_reckoning.DeadReckoner` and
its EKF subclasses) at whatever sample rate is being tested.
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

    def evaluate(self, velocity0: np.ndarray, position0: np.ndarray, dt: float):
        """
        Closed-form velocity/position after ``dt`` seconds of this
        segment's constant acceleration, plus the (constant) true nav-frame
        acceleration at that instant.

        """
        velocity = velocity0 + self.accel_nav * dt
        position = position0 + velocity0 * dt + 0.5 * self.accel_nav * dt**2

        return velocity, position, self.accel_nav


class CoordinatedTurnSegment:
    """
    A constant-speed, constant-turn-rate ("coordinated turn") segment:
    exact closed-form circular motion in the horizontal (x-y) plane. Speed
    and initial heading are taken from the velocity the trajectory has on
    entering this segment (so segments chain naturally - just specify the
    turn rate); the vertical velocity component is held constant, so this
    primitive models level turns.

    Args:
        * duration {``float``} -- Length of this segment, in seconds. Must
          be positive.
        * turn_rate_deg {``float``} -- Constant turn rate about the
          vertical (gravity) axis, in degrees/s. Positive turns the
          horizontal velocity from ``+x`` toward ``+y``.

    """

    def __init__(self, duration: float, turn_rate_deg: float):
        if duration <= 0:
            raise ValueError(f"duration must be positive, got {duration}")

        self.duration = float(duration)
        self.turn_rate_deg = float(turn_rate_deg)
        self.angular_velocity_body = np.array([0.0, 0.0, turn_rate_deg])

    def evaluate(self, velocity0: np.ndarray, position0: np.ndarray, dt: float):
        """
        Closed-form velocity/position after ``dt`` seconds of this
        segment's constant-speed turn, plus the true (centripetal)
        nav-frame acceleration at that instant.

        """
        omega = np.radians(self.turn_rate_deg)
        vx0, vy0, vz0 = velocity0
        speed_xy = np.hypot(vx0, vy0)
        heading0 = np.arctan2(vy0, vx0) if speed_xy > 0 else 0.0

        if abs(omega) < 1e-12:
            velocity = np.array([vx0, vy0, vz0])
            position = position0 + velocity0 * dt
            accel_nav = np.zeros(3)

            return velocity, position, accel_nav

        heading = heading0 + omega * dt
        velocity = np.array([speed_xy * np.cos(heading), speed_xy * np.sin(heading), vz0])
        position = position0 + np.array(
            [
                speed_xy / omega * (np.sin(heading) - np.sin(heading0)),
                -speed_xy / omega * (np.cos(heading) - np.cos(heading0)),
                vz0 * dt,
            ]
        )
        accel_nav = speed_xy * omega * np.array([-np.sin(heading), np.cos(heading), 0.0])

        return velocity, position, accel_nav


def _propagate(
    attitude: Quaternion,
    velocity: np.ndarray,
    position: np.ndarray,
    segment,
    dt: float,
):
    """
    Propagate attitude/velocity/position over ``dt`` seconds of
    ``segment``, and return the true nav-frame acceleration at that
    instant, using ``segment``'s own closed-form :meth:`evaluate`.

    """
    rate_rad = np.radians(segment.angular_velocity_body)
    rate = np.linalg.norm(rate_rad)

    if rate > 0:
        delta = Quaternion.from_axis_angle(rate_rad / rate, rate * dt)
        attitude = attitude * delta
        attitude = attitude * (1.0 / abs(attitude))

    new_velocity, new_position, accel_nav = segment.evaluate(velocity, position, dt)

    return attitude, new_velocity, new_position, accel_nav


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
    a chain of segments (:class:`MotionSegment` and/or
    :class:`CoordinatedTurnSegment`).

    Args:
        * segments {``list``} -- The segments making up this trajectory, in
          order. Must be non-empty. Each must expose ``duration``,
          ``angular_velocity_body`` and an
          ``evaluate(velocity0, position0, dt)`` method, as
          :class:`MotionSegment` and :class:`CoordinatedTurnSegment` do.
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
        segments: list,
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
            attitude, velocity, position, _accel_nav = _propagate(
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

        attitude, velocity, position, accel_nav = _propagate(q0, v0, p0, segment, elapsed)

        return segment, attitude, velocity, position, accel_nav

    def state_at(self, t: float) -> TrajectoryState:
        """
        The exact ground-truth :class:`TrajectoryState` at time ``t``.

        Args:
            * t {``float``} -- Time, in seconds, within ``[0, self.duration]``.

        Returns:
            * {``TrajectoryState``}

        """
        _segment, attitude, velocity, position, accel_nav = self._evaluate(t)

        return TrajectoryState(
            t=t,
            attitude=attitude,
            accel_nav=accel_nav,
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
        segment, attitude, _velocity, _position, accel_nav = self._evaluate(t)
        accel_body = _nav_accel_to_body_frame(
            accel_nav, attitude, self.gravity_magnitude, self.gravity_direction
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
