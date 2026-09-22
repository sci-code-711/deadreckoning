"""Benchmark harness: run a reconstruction algorithm against a synthetic
trajectory's known ground truth.
"""

from dataclasses import dataclass

import numpy as np

from .metrics import attitude_angle_error_deg, position_errors
from .noise import add_gaussian_noise
from .samples import TrajectoryState
from .synthetic import SyntheticTrajectory


@dataclass
class BenchmarkResult:
    """
    The outcome of running a reckoner against a
    :class:`deadrec.synthetic.SyntheticTrajectory`'s known ground truth.

    Args:
        * ground_truth {``list[TrajectoryState]``} -- The trajectory's exact
          ground-truth state at each sampled time.
        * reconstructed {``list[TrajectoryState]``} -- The reckoner's
          reconstructed state at each sampled time, same length and order
          as ``ground_truth``.
        * position_rmse {``float``} -- Root-mean-square position error
          across every sampled time.
        * final_position_error {``float``} -- Position error at the last
          sampled time.
        * final_attitude_error {``float``} -- Angle, in degrees
          (``[0, 180]``), between the reconstructed and ground-truth
          attitude at the last sampled time.

    """

    ground_truth: list[TrajectoryState]
    reconstructed: list[TrajectoryState]
    position_rmse: float
    final_position_error: float
    final_attitude_error: float


def run_benchmark(
    trajectory: SyntheticTrajectory,
    reckoner_factory,
    *,
    hz: float,
    accel_std: float = 0.0,
    gyro_std: float = 0.0,
    seed=None,
) -> BenchmarkResult:
    """
    Run a reconstruction algorithm against ``trajectory``'s known ground
    truth, at a given sample rate and, optionally, with injected Gaussian
    noise.

    Args:
        * trajectory {``SyntheticTrajectory``} -- The known trajectory to
          benchmark against, e.g. from :data:`deadrec.benchmark_cases.BENCHMARK_CASES`.
        * reckoner_factory {``Callable``} -- Builds the reckoner to
          benchmark, called as ``reckoner_factory(initial_attitude,
          gravity_magnitude, gravity_direction=..., initial_velocity=...,
          initial_position=...)``. Any of
          :class:`deadrec.dead_reckoning.DeadReckoner`,
          :class:`deadrec.ekf.GravityCorrectedEKF` or
          :class:`deadrec.ekf.WindowedGravityCorrectedEKF` can be passed
          directly - they all accept this signature and this harness only
          ever calls ``.run()``, which every one of them supports.
        * hz {``float``} -- Sample rate to synthesize readings at.
        * accel_std, gyro_std {``float``} -- Gaussian noise standard
          deviation to inject, see :func:`deadrec.noise.add_gaussian_noise`.
          Default to no noise.
        * seed -- Noise RNG seed, see :func:`deadrec.noise.add_gaussian_noise`.

    Returns:
        * {``BenchmarkResult``}

    """
    ground_truth, samples = trajectory.sample_at_rate(hz)

    if accel_std or gyro_std:
        samples = add_gaussian_noise(samples, accel_std=accel_std, gyro_std=gyro_std, seed=seed)

    reckoner = reckoner_factory(
        ground_truth[0].attitude,
        trajectory.gravity_magnitude,
        gravity_direction=trajectory.gravity_direction,
        initial_velocity=ground_truth[0].velocity,
        initial_position=ground_truth[0].position,
    )
    reconstructed = reckoner.run(samples)

    errors = position_errors(reconstructed, [g.position for g in ground_truth])

    return BenchmarkResult(
        ground_truth=ground_truth,
        reconstructed=reconstructed,
        position_rmse=float(np.sqrt(np.mean(errors**2))),
        final_position_error=float(errors[-1]),
        final_attitude_error=attitude_angle_error_deg(
            reconstructed[-1].attitude, ground_truth[-1].attitude
        ),
    )
