"""Deterministic synthetic IMU trajectory generation.

Produces synthetic IMU samples for a handful of named trajectories whose
true position, velocity and attitude are known in closed form, plus the
matching ground-truth trajectory states - for validating
``deadrec.dead_reckoning``/``deadrec.ekf`` against a known-correct answer
instead of only eyeballing real recordings.

This module deliberately depends only on ``deadrec.quaternion`` (generic
quaternion algebra) and ``deadrec.samples`` (plain data containers), never on
``deadrec.kinematics``/``deadrec.attitude``/``deadrec.dead_reckoning``/
``deadrec.ekf`` - so using it to validate those modules doesn't just test
them against themselves.

"""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .quaternion import Quaternion
from .samples import ImuSample, TrajectoryState

STANDARD_GRAVITY = 9.80665

Vec3Fn = Callable[[float], np.ndarray]
ScalarFn = Callable[[float], float]


@dataclass
class YawOnlyTrajectory:
    """
    A trajectory whose attitude is a pure yaw rotation (about the shared
    navigation/body Z axis), with position/velocity/acceleration given in
    closed form. Every trajectory in :data:`TRAJECTORIES` is built this way,
    which keeps the true accelerometer/gyroscope readings exactly
    derivable, with no numerical differentiation involved.

    Args:
        * name {``str``} -- The trajectory's name.
        * position {``Callable[[float], array-like]``} -- True nav-frame
          position at time ``t``.
        * velocity {``Callable[[float], array-like]``} -- True nav-frame
          velocity at time ``t``.
        * acceleration {``Callable[[float], array-like]``} -- True nav-frame
          linear acceleration (gravity excluded) at time ``t``.
        * yaw {``Callable[[float], float]``} -- True yaw angle (radians) at
          time ``t``.
        * yaw_rate {``Callable[[float], float]``} -- True yaw rate
          (radians/s) at time ``t``.

    """

    name: str
    position: Vec3Fn
    velocity: Vec3Fn
    acceleration: Vec3Fn
    yaw: ScalarFn
    yaw_rate: ScalarFn

    def attitude(self, t: float) -> Quaternion:
        """The true body-to-navigation attitude quaternion at time ``t``.

        Built directly from the standard half-angle formula for a
        single-axis rotation, rather than via
        :meth:`Quaternion.from_eul_angles` - that method's ``n`` argument is
        the rotation angle *before* halving (see its own tests:
        ``from_eul_angles(0, 0, pi/2) == Quaternion(0, 0, 0, 1)``, a 180
        degree rotation), so passing the true yaw angle straight through
        would silently produce a quaternion for twice that rotation.

        """
        half_yaw = self.yaw(t) / 2.0
        return Quaternion(np.cos(half_yaw), 0.0, 0.0, np.sin(half_yaw))

    def angular_velocity_deg(self, t: float) -> np.ndarray:
        """The true body-frame angular velocity (degrees/s) at time ``t``."""
        return np.degrees([0.0, 0.0, self.yaw_rate(t)])


def stationary() -> YawOnlyTrajectory:
    """A trajectory that never moves or rotates."""
    zero = np.zeros(3)
    return YawOnlyTrajectory(
        name="stationary",
        position=lambda t: zero,
        velocity=lambda t: zero,
        acceleration=lambda t: zero,
        yaw=lambda t: 0.0,
        yaw_rate=lambda t: 0.0,
    )


def constant_velocity(velocity=(1.0, 0.0, 0.0)) -> YawOnlyTrajectory:
    """A straight line travelled at a constant, non-rotating velocity."""
    v = np.asarray(velocity, dtype=float)
    zero = np.zeros(3)
    return YawOnlyTrajectory(
        name="constant_velocity",
        position=lambda t: v * t,
        velocity=lambda t: v,
        acceleration=lambda t: zero,
        yaw=lambda t: 0.0,
        yaw_rate=lambda t: 0.0,
    )


def constant_acceleration(acceleration=(1.0, 0.0, 0.0)) -> YawOnlyTrajectory:
    """A straight line travelled under constant, non-rotating acceleration,
    starting from rest."""
    a = np.asarray(acceleration, dtype=float)
    return YawOnlyTrajectory(
        name="constant_acceleration",
        position=lambda t: 0.5 * a * t**2,
        velocity=lambda t: a * t,
        acceleration=lambda t: a,
        yaw=lambda t: 0.0,
        yaw_rate=lambda t: 0.0,
    )


def pure_rotation(rate_deg_s: float = 45.0) -> YawOnlyTrajectory:
    """An in-place spin at a constant body yaw rate, with no translation."""
    rate = np.radians(rate_deg_s)
    zero = np.zeros(3)
    return YawOnlyTrajectory(
        name="pure_rotation",
        position=lambda t: zero,
        velocity=lambda t: zero,
        acceleration=lambda t: zero,
        yaw=lambda t: rate * t,
        yaw_rate=lambda t: rate,
    )


def circular_turn(speed: float = 5.0, radius: float = 20.0) -> YawOnlyTrajectory:
    """A horizontal circle at constant speed, nose tangent to the path (a
    constant yaw rate of ``speed / radius``)."""
    omega = speed / radius
    centripetal = speed * omega

    return YawOnlyTrajectory(
        name="circular_turn",
        position=lambda t: np.array(
            [radius * np.sin(omega * t), radius * (1 - np.cos(omega * t)), 0.0]
        ),
        velocity=lambda t: np.array([speed * np.cos(omega * t), speed * np.sin(omega * t), 0.0]),
        acceleration=lambda t: np.array(
            [-centripetal * np.sin(omega * t), centripetal * np.cos(omega * t), 0.0]
        ),
        yaw=lambda t: omega * t,
        yaw_rate=lambda t: omega,
    )


def climbing_helix(
    speed: float = 5.0, radius: float = 20.0, climb_rate: float = 0.5
) -> YawOnlyTrajectory:
    """A :func:`circular_turn` with a constant climb rate added on top."""
    base = circular_turn(speed=speed, radius=radius)
    climb = np.array([0.0, 0.0, climb_rate])

    return YawOnlyTrajectory(
        name="climbing_helix",
        position=lambda t: base.position(t) + climb * t,
        velocity=lambda t: base.velocity(t) + climb,
        acceleration=base.acceleration,
        yaw=base.yaw,
        yaw_rate=base.yaw_rate,
    )


TRAJECTORIES: dict[str, Callable[..., YawOnlyTrajectory]] = {
    "stationary": stationary,
    "constant_velocity": constant_velocity,
    "constant_acceleration": constant_acceleration,
    "pure_rotation": pure_rotation,
    "circular_turn": circular_turn,
    "climbing_helix": climbing_helix,
}


def _rotate_into_body(attitude: Quaternion, vector_nav) -> np.ndarray:
    """
    Rotate a navigation-frame vector into the body frame - the inverse of
    the ``attitude * v * attitude.conjugate()`` rotation that
    ``deadrec.dead_reckoning.accel_to_nav_frame`` uses to bring body-frame
    accelerometer readings into the navigation frame.

    """
    nav_quat = Quaternion(0.0, *vector_nav)
    body_quat = attitude.conjugate() * nav_quat * attitude
    return np.array([body_quat.x, body_quat.y, body_quat.z])


def sample_trajectory(
    trajectory: YawOnlyTrajectory,
    *,
    duration_s: float,
    rate_hz: float,
    noise_std_accel: float = 0.0,
    noise_std_gyro: float = 0.0,
    seed: int = 0,
) -> tuple[list[ImuSample], list[TrajectoryState]]:
    """
    Sample a trajectory into synthetic IMU readings and the matching
    ground-truth trajectory states.

    Args:
        * trajectory {``YawOnlyTrajectory``} -- The trajectory to sample,
          e.g. from :data:`TRAJECTORIES`.
        * duration_s {``float``} -- Total duration to sample, in seconds.
        * rate_hz {``float``} -- Sample rate, in Hz.
        * noise_std_accel {``float``} -- Standard deviation of Gaussian
          noise added to accelerometer readings (never to the ground
          truth). Defaults to ``0.0`` (noiseless, exact readings).
        * noise_std_gyro {``float``} -- Standard deviation of Gaussian
          noise added to gyroscope readings (never to the ground truth).
          Defaults to ``0.0``.
        * seed {``int``} -- Seed for the noise random number generator. The
          same seed always reproduces identical output. Only relevant when
          a noise standard deviation is non-zero.

    Returns:
        * {``tuple[list[ImuSample], list[TrajectoryState]]``} -- The
          synthetic readings and the matching ground-truth states, one pair
          per sample time.

    """
    if duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s}")
    if rate_hz <= 0:
        raise ValueError(f"rate_hz must be positive, got {rate_hz}")

    dt = 1.0 / rate_hz
    sample_count = int(round(duration_s * rate_hz)) + 1
    times = np.arange(sample_count) * dt
    rng = np.random.default_rng(seed)

    samples = []
    truth = []
    for t in times:
        attitude = trajectory.attitude(t)
        accel_nav = trajectory.acceleration(t)
        accel_reading = _rotate_into_body(
            attitude, accel_nav + np.array([0.0, 0.0, STANDARD_GRAVITY])
        )
        gyro_reading = trajectory.angular_velocity_deg(t)

        if noise_std_accel:
            accel_reading = accel_reading + rng.normal(0.0, noise_std_accel, 3)
        if noise_std_gyro:
            gyro_reading = gyro_reading + rng.normal(0.0, noise_std_gyro, 3)

        samples.append(ImuSample(t=t, accel=accel_reading, gyro=gyro_reading))
        truth.append(
            TrajectoryState(
                t=t,
                attitude=attitude,
                accel_nav=accel_nav,
                velocity=trajectory.velocity(t),
                position=trajectory.position(t),
                euler=attitude.to_euler_angles(),
            )
        )

    return samples, truth
