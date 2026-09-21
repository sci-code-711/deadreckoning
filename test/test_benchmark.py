import math

import pytest

from deadrec.benchmark import BenchmarkResult, run_benchmark
from deadrec.benchmark_cases import BENCHMARK_CASES
from deadrec.dead_reckoning import DeadReckoner
from deadrec.ekf import GravityCorrectedEKF, WindowedGravityCorrectedEKF

_SIMPLE_CASES = ["stationary", "constant_rotation", "constant_linear_acceleration"]
_COMPOSITE_CASES = ["circular_motion", "composite_sequence", "full_composite"]
_RECKONERS = [DeadReckoner, GravityCorrectedEKF, WindowedGravityCorrectedEKF]


def test_run_benchmark_pairs_ground_truth_and_reconstructed_states():
    trajectory = BENCHMARK_CASES["constant_linear_acceleration"]()

    result = run_benchmark(trajectory, DeadReckoner, hz=50)

    assert isinstance(result, BenchmarkResult)
    assert len(result.reconstructed) == len(result.ground_truth)
    assert [s.t for s in result.reconstructed] == [s.t for s in result.ground_truth]


@pytest.mark.parametrize("name", _SIMPLE_CASES)
def test_noiseless_simple_cases_reconstruct_almost_exactly(name):
    trajectory = BENCHMARK_CASES[name]()

    result = run_benchmark(trajectory, DeadReckoner, hz=50)

    assert result.position_rmse < 1e-6
    assert result.final_position_error < 1e-6
    assert result.final_attitude_error < 1e-6


@pytest.mark.parametrize("name", _COMPOSITE_CASES)
def test_noiseless_composite_cases_stay_small_and_bounded(name):
    trajectory = BENCHMARK_CASES[name]()

    result = run_benchmark(trajectory, DeadReckoner, hz=100)

    # These involve rotating acceleration, where the reckoner's discrete
    # trapezoidal integration isn't exact even error-free - error should
    # still be tiny relative to the trajectory's scale (tens of metres).
    assert result.position_rmse < 0.5
    assert result.final_position_error < 1.0
    assert result.final_attitude_error < 0.05


def test_noise_increases_position_error():
    trajectory = BENCHMARK_CASES["circular_motion"]()

    noiseless = run_benchmark(trajectory, DeadReckoner, hz=100)
    noisy = run_benchmark(trajectory, DeadReckoner, hz=100, accel_std=0.1, gyro_std=0.5, seed=0)

    assert noisy.position_rmse > noiseless.position_rmse
    assert math.isfinite(noisy.position_rmse)


def test_noisy_run_is_reproducible_with_same_seed():
    trajectory = BENCHMARK_CASES["composite_sequence"]()

    first = run_benchmark(trajectory, DeadReckoner, hz=50, accel_std=0.2, gyro_std=1.0, seed=11)
    second = run_benchmark(trajectory, DeadReckoner, hz=50, accel_std=0.2, gyro_std=1.0, seed=11)

    assert first.position_rmse == pytest.approx(second.position_rmse)
    assert first.final_position_error == pytest.approx(second.final_position_error)


@pytest.mark.parametrize("reckoner_factory", _RECKONERS)
def test_run_benchmark_works_with_every_reckoner_class(reckoner_factory):
    trajectory = BENCHMARK_CASES["constant_rotation"]()

    result = run_benchmark(
        trajectory, reckoner_factory, hz=50, accel_std=0.05, gyro_std=0.3, seed=2
    )

    assert math.isfinite(result.position_rmse)
    assert math.isfinite(result.final_position_error)
    assert math.isfinite(result.final_attitude_error)
    assert result.position_rmse >= 0
    assert result.final_attitude_error >= 0


def test_run_benchmark_seeds_reckoner_with_trajectory_initial_conditions():
    # circular_motion starts with a nonzero initial_velocity - the reckoner
    # must be seeded with it, not assume a zero start.
    trajectory = BENCHMARK_CASES["circular_motion"]()

    result = run_benchmark(trajectory, DeadReckoner, hz=100)

    assert result.position_rmse < 0.01
