"""Shared data types for IMU samples and reconstructed trajectory state."""

from dataclasses import dataclass, field

import numpy as np

from .quaternion import Quaternion


def _as_vec3(value, name: str) -> np.ndarray:
    vec = np.asarray(value, dtype=float)
    if vec.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {vec.shape}")
    return vec


@dataclass
class ImuSample:
    """A single timestamped IMU reading.

    Args:
        * t {``float``} -- Timestamp of the reading.
        * accel {``array-like``} -- Accelerometer reading (ax, ay, az).
        * gyro {``array-like``} -- Gyroscope reading in degrees/s (vl, vm, vn).

    """

    t: float
    accel: np.ndarray
    gyro: np.ndarray

    def __post_init__(self):
        self.accel = _as_vec3(self.accel, "accel")
        self.gyro = _as_vec3(self.gyro, "gyro")


@dataclass
class TrajectoryState:
    """The reconstructed state of the system at a single point in time.

    Args:
        * t {``float``} -- Timestamp of this state.
        * attitude {``Quaternion``} -- Attitude of the system relative to the
          navigation frame.
        * accel_nav {``array-like``} -- Linear acceleration in the navigation
          frame, with gravity removed.
        * velocity {``array-like``} -- Linear velocity in the navigation frame.
        * position {``array-like``} -- Linear displacement in the navigation frame.
        * euler {``tuple``} -- Attitude as Euler angles, equivalent to `attitude`.

    """

    t: float
    attitude: Quaternion
    accel_nav: np.ndarray
    velocity: np.ndarray
    position: np.ndarray
    euler: tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))

    def __post_init__(self):
        if not isinstance(self.attitude, Quaternion):
            raise TypeError(f"attitude must be a Quaternion, got {type(self.attitude)}")

        self.accel_nav = _as_vec3(self.accel_nav, "accel_nav")
        self.velocity = _as_vec3(self.velocity, "velocity")
        self.position = _as_vec3(self.position, "position")
