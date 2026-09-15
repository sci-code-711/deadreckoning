"""Gravity-vector based attitude estimation."""

import numpy as np

from .quaternion import Quaternion


def attitude_aligning_vectors(source, target=(0, 0, 1)) -> Quaternion:
    """
    Find the attitude quaternion that rotates the direction ``source`` onto
    the direction ``target``.

    Args:
        * source {``array-like``} -- The 3-vector to rotate (e.g. a raw
          accelerometer reading). Does not need to be normalised.
        * target {``array-like``} -- The 3-vector to rotate onto. Defaults to
          ``(0, 0, 1)``.

    Returns:
        * {``Quaternion``} -- The rotation quaternion. When ``source`` and
          ``target`` are parallel or anti-parallel the cross product is zero
          and the identity quaternion is returned - this includes the case
          where the vectors point in exactly opposite directions, which is
          a known limitation of this method.

    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)

    source = source / np.linalg.norm(source)
    cross_product = np.cross(source, target)
    cross_norm = np.linalg.norm(cross_product)

    if cross_norm == 0:
        return Quaternion(1, 0, 0, 0)

    axis = cross_product / cross_norm
    phi = np.arctan(cross_norm / np.dot(source, target))

    if phi < 0:
        phi = phi + np.pi

    return Quaternion(
        np.cos(phi / 2),
        axis[0] * np.sin(phi / 2),
        axis[1] * np.sin(phi / 2),
        axis[2] * np.sin(phi / 2),
    )


def initial_attitude_from_gravity(accel_samples, *, gravity_direction=(0, 0, 1)) -> Quaternion:
    """
    Estimate the initial attitude of the system by averaging the rotation
    that aligns each of several stationary accelerometer readings with
    ``gravity_direction``.

    Averages via the eigenvector method (Markley et al.): forms the mean
    outer product of each sample's rotation quaternion with itself, and
    takes the eigenvector of the largest eigenvalue as the average attitude.

    Pass only the stationary readings that should contribute to the
    estimate, e.g. the first N samples of a calibration period.

    Args:
        * accel_samples {``array-like``} -- An (N, 3) array of stationary
          accelerometer readings.
        * gravity_direction {``array-like``} -- The reference "down"
          direction to align samples with. Defaults to ``(0, 0, 1)``.

    Returns:
        * {``Quaternion``} -- The averaged initial attitude estimate.

    """
    accel_samples = np.asarray(accel_samples, dtype=float)
    if accel_samples.ndim != 2 or accel_samples.shape[1] != 3:
        raise ValueError(f"accel_samples must have shape (N, 3), got {accel_samples.shape}")

    n = accel_samples.shape[0]
    mat = np.zeros((4, 4))

    for sample in accel_samples:
        q = attitude_aligning_vectors(sample, gravity_direction)
        qv = np.array([q.w, q.x, q.y, q.z])
        mat = mat + np.outer(qv, qv) / n

    eig_val, eig_vec = np.linalg.eig(mat)
    q = eig_vec[:, np.argmax(eig_val)].real
    q = q / np.linalg.norm(q)

    return Quaternion(*q)


def gravity_deviation(accel_nav) -> float:
    """
    The magnitude of a gravity-removed, navigation-frame acceleration
    reading, used to gate the EKF gravity-correction step: a large value
    indicates real (non-gravitational) acceleration, during which the
    gravity-vector correction should not be trusted.

    Args:
        * accel_nav {``array-like``} -- Gravity-removed acceleration in the
          navigation frame.

    Returns:
        * {``float``} -- The deviation magnitude.

    """
    return float(np.linalg.norm(np.asarray(accel_nav, dtype=float)))
