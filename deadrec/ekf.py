"""Attitude correction using gravity-vector measurements."""

import numpy as np

from .attitude import attitude_aligning_vectors, gravity_deviation
from .dead_reckoning import DeadReckoner, accel_to_nav_frame
from .quaternion import Quaternion
from .samples import ImuSample


def _heading_only(attitude: Quaternion) -> Quaternion:
    """
    Extract the component of ``attitude`` that rotates about the gravity
    axis (its "heading"), discarding tilt. Assumes gravity is along z, i.e.
    the default ``gravity_direction=(0, 0, 1)`` convention used elsewhere in
    this package.

    Args:
        * attitude {``Quaternion``} -- The attitude to extract heading from.

    Returns:
        * {``Quaternion``} -- The heading-only component.

    """
    norm = np.sqrt(attitude.w**2 + attitude.z**2)
    return Quaternion(attitude.w / norm, 0, 0, attitude.z / norm)


def _blend_attitudes(a: Quaternion, b: Quaternion, weight_b: float) -> Quaternion:
    """
    Average two attitude quaternions via the eigenvector method (as used in
    :func:`deadrec.attitude.initial_attitude_from_gravity`, but for two
    unevenly-weighted quaternions instead of many equally-weighted ones).

    Args:
        * a {``Quaternion``} -- The first attitude, with weight ``1 - weight_b``.
        * b {``Quaternion``} -- The second attitude, with weight ``weight_b``.
        * weight_b {``float``} -- Weight given to ``b``, between 0 and 1.

    Returns:
        * {``Quaternion``} -- The blended attitude.

    """
    av = np.array([a.w, a.x, a.y, a.z])
    bv = np.array([b.w, b.x, b.y, b.z])
    mat = np.outer(av, av) * (1 - weight_b) + np.outer(bv, bv) * weight_b

    eig_val, eig_vec = np.linalg.eig(mat)
    q = eig_vec[:, np.argmax(eig_val)].real

    return Quaternion(*(q / np.linalg.norm(q)))


class GravityCorrectedEKF(DeadReckoner):
    """
    A :class:`DeadReckoner` that additionally fuses in a gravity-vector
    attitude correction at each step: whenever the predicted, gravity-removed
    acceleration is small enough to plausibly be measurement noise (rather
    than real acceleration), the predicted attitude is blended with an
    attitude estimated directly from the current accelerometer reading
    (assuming the system is stationary), correcting for gyroscope drift.

    Args:
        * initial_attitude, gravity_magnitude, gravity_direction,
          initial_velocity, initial_position -- See :class:`DeadReckoner`.
        * deviation_threshold {``float``} -- Maximum
          :func:`deadrec.attitude.gravity_deviation` at which the gravity
          correction is applied; above this, only the gyroscope-predicted
          attitude is used for the step. Defaults to ``0.02``.
        * beta {``float``} -- Weight given to the gravity-derived attitude
          when blending it with the predicted attitude (0 = ignore it
          entirely, 1 = use it exclusively). Defaults to ``0.2``.

    """

    def __init__(
        self,
        initial_attitude: Quaternion,
        gravity_magnitude: float,
        *,
        gravity_direction=(0, 0, 1),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_position=(0.0, 0.0, 0.0),
        deviation_threshold: float = 0.02,
        beta: float = 0.2,
    ):
        super().__init__(
            initial_attitude,
            gravity_magnitude,
            gravity_direction=gravity_direction,
            initial_velocity=initial_velocity,
            initial_position=initial_position,
        )
        self.deviation_threshold = deviation_threshold
        self.beta = beta

    def _predict_attitude(self, sample: ImuSample, dt: float) -> Quaternion:
        predicted = super()._predict_attitude(sample, dt)
        predicted_accel_nav = accel_to_nav_frame(
            sample.accel, predicted, self.gravity_magnitude, self.gravity_direction
        )

        if gravity_deviation(predicted_accel_nav) >= self.deviation_threshold:
            return predicted

        gravity_estimate = _heading_only(predicted) * attitude_aligning_vectors(
            sample.accel, self.gravity_direction
        )

        return _blend_attitudes(predicted, gravity_estimate, self.beta)
