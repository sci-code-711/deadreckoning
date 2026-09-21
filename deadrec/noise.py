"""Gaussian noise injection for synthetic IMU reading streams."""

import numpy as np

from .samples import ImuSample


def add_gaussian_noise(
    samples: list[ImuSample], *, accel_std: float = 0.0, gyro_std: float = 0.0, seed=None
) -> list[ImuSample]:
    """
    Add independent Gaussian white noise to a sequence of IMU readings, to
    simulate sensor noise on a noiseless synthetic trajectory (e.g. from
    :class:`deadrec.synthetic.SyntheticTrajectory`).

    Args:
        * samples {``list[ImuSample]``} -- The (typically noiseless)
          readings to add noise to. Left unmodified; new samples are
          returned.
        * accel_std {``float``} -- Standard deviation of the noise added to
          each accelerometer axis, in the same units as ``sample.accel``.
          Defaults to ``0.0`` (no accelerometer noise).
        * gyro_std {``float``} -- Standard deviation of the noise added to
          each gyroscope axis, in degrees/s. Defaults to ``0.0`` (no
          gyroscope noise).
        * seed {``int``, ``np.random.Generator`` or ``None``} -- Seed (or
          generator) for reproducible noise, passed to
          :func:`numpy.random.default_rng`. Defaults to ``None`` (a
          non-reproducible seed).

    Returns:
        * {``list[ImuSample]``} -- New samples, each with independent
          ``N(0, std)`` noise added per axis. Timestamps are unchanged.

    """
    rng = np.random.default_rng(seed)
    n = len(samples)

    accel_noise = rng.normal(scale=accel_std, size=(n, 3))
    gyro_noise = rng.normal(scale=gyro_std, size=(n, 3))

    return [
        ImuSample(t=sample.t, accel=sample.accel + accel_noise[i], gyro=sample.gyro + gyro_noise[i])
        for i, sample in enumerate(samples)
    ]
