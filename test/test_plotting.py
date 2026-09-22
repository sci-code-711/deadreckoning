import matplotlib
import pytest

matplotlib.use("Agg")

from deadrec.benchmark import run_benchmark  # noqa: E402
from deadrec.benchmark_cases import BENCHMARK_CASES  # noqa: E402
from deadrec.dead_reckoning import DeadReckoner  # noqa: E402
from deadrec.ekf import GravityCorrectedEKF  # noqa: E402
from deadrec.plotting import plot_benchmark_results  # noqa: E402


def test_plot_benchmark_results_with_a_single_result():
    trajectory = BENCHMARK_CASES["circular_motion"]()
    result = run_benchmark(trajectory, DeadReckoner, hz=50)

    fig = plot_benchmark_results(result)

    assert len(fig.axes) == 3
    trajectory_ax, position_ax, attitude_ax = fig.axes
    # Ground truth + the one reconstruction on the trajectory panel.
    assert len(trajectory_ax.lines) == 2
    assert len(position_ax.lines) == 1
    assert len(attitude_ax.lines) == 1

    matplotlib.pyplot.close(fig)


def test_plot_benchmark_results_compares_multiple_reckoners():
    trajectory = BENCHMARK_CASES["constant_rotation"]()
    results = {
        "DeadReckoner": run_benchmark(trajectory, DeadReckoner, hz=50, gyro_std=0.5, seed=1),
        "GravityCorrectedEKF": run_benchmark(
            trajectory, GravityCorrectedEKF, hz=50, gyro_std=0.5, seed=1
        ),
    }

    fig = plot_benchmark_results(results, title="constant_rotation")

    trajectory_ax, position_ax, attitude_ax = fig.axes
    assert len(trajectory_ax.lines) == 3  # ground truth + 2 reckoners
    assert len(position_ax.lines) == 2
    assert len(attitude_ax.lines) == 2

    trajectory_labels = [line.get_label() for line in trajectory_ax.lines]
    assert "Ground truth" in trajectory_labels
    assert "DeadReckoner" in trajectory_labels
    assert "GravityCorrectedEKF" in trajectory_labels

    matplotlib.pyplot.close(fig)


def test_plot_benchmark_results_rejects_empty_input():
    with pytest.raises(ValueError):
        plot_benchmark_results({})


def test_plot_benchmark_results_rejects_mismatched_ground_truth():
    short_trajectory = BENCHMARK_CASES["stationary"]()
    long_trajectory = BENCHMARK_CASES["constant_rotation"]()
    results = {
        "a": run_benchmark(short_trajectory, DeadReckoner, hz=50),
        "b": run_benchmark(long_trajectory, DeadReckoner, hz=50),
    }

    with pytest.raises(ValueError):
        plot_benchmark_results(results)
