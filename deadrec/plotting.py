"""Visualizing benchmark results: known trajectory vs. reconstruction(s),
plus position/attitude error over time.
"""

import matplotlib.pyplot as plt
import numpy as np

from .benchmark import BenchmarkResult

# Ground truth is always this color; reconstructions take the next colors in
# this fixed order, so a given name (e.g. "GravityCorrectedEKF") always gets
# the same color across figures, rather than an arbitrary cycled hue.
_TRUTH_COLOR = "#2a78d6"
_SERIES_COLORS = ["#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7", "#e34948"]


def _position_errors(result: BenchmarkResult) -> np.ndarray:
    return np.array(
        [
            np.linalg.norm(r.position - g.position)
            for r, g in zip(result.reconstructed, result.ground_truth)
        ]
    )


def _attitude_errors_deg(result: BenchmarkResult) -> np.ndarray:
    errors = []
    for r, g in zip(result.reconstructed, result.ground_truth):
        relative = r.attitude.conjugate() * g.attitude
        relative = relative * (1.0 / abs(relative))
        w = np.clip(abs(relative.w), -1.0, 1.0)
        errors.append(np.degrees(2 * np.arccos(w)))

    return np.array(errors)


def plot_benchmark_results(
    results: BenchmarkResult | dict[str, BenchmarkResult], *, title: str | None = None
):
    """
    Plot one or more :class:`deadrec.benchmark.BenchmarkResult`'s
    reconstructed trajectory against their shared ground truth, alongside
    position and attitude error over time.

    Args:
        * results {``BenchmarkResult`` or ``dict[str, BenchmarkResult]``} --
          A single result, or several keyed by a label for the reckoner
          that produced them (e.g. ``{"DeadReckoner": ..., "GravityCorrectedEKF": ...}``),
          so multiple reckoners can be compared on the same trajectory in
          one figure. All results must share the same ground truth (e.g.
          from running each reckoner against the same
          :func:`deadrec.benchmark.run_benchmark` trajectory/rate).
        * title {``str``} -- Optional figure title.

    Returns:
        * {``matplotlib.figure.Figure``}

    """
    if isinstance(results, BenchmarkResult):
        results = {"reconstructed": results}

    if not results:
        raise ValueError("results must contain at least one BenchmarkResult")

    ground_truth = next(iter(results.values())).ground_truth
    for name, result in results.items():
        if len(result.ground_truth) != len(ground_truth):
            raise ValueError(
                f"results don't share the same ground truth: {name!r} has "
                f"{len(result.ground_truth)} samples, expected {len(ground_truth)}"
            )

    fig = plt.figure(figsize=(16, 5.5))
    if title:
        fig.suptitle(title, fontsize=13, fontweight="bold")

    ax_traj = fig.add_subplot(1, 3, 1, projection="3d")
    truth_pos = np.array([s.position for s in ground_truth])
    ax_traj.plot(
        *truth_pos.T, color=_TRUTH_COLOR, linestyle="--", linewidth=2.5, label="Ground truth"
    )

    for i, (name, result) in enumerate(results.items()):
        color = _SERIES_COLORS[i % len(_SERIES_COLORS)]
        pos = np.array([s.position for s in result.reconstructed])
        ax_traj.plot(*pos.T, color=color, linewidth=1.8, label=name)

    ax_traj.set_xlabel("X (m)")
    ax_traj.set_ylabel("Y (m)")
    ax_traj.set_zlabel("Z (m)")
    ax_traj.set_title("Trajectory")
    ax_traj.legend(loc="upper left", fontsize=8, frameon=False)

    t = np.array([s.t for s in ground_truth])

    ax_pos_err = fig.add_subplot(1, 3, 2)
    for i, (name, result) in enumerate(results.items()):
        color = _SERIES_COLORS[i % len(_SERIES_COLORS)]
        errors = np.maximum(_position_errors(result), 1e-12)
        ax_pos_err.plot(t, errors, color=color, linewidth=2, label=name)

    ax_pos_err.set_yscale("log")
    ax_pos_err.set_xlabel("Time (s)")
    ax_pos_err.set_ylabel("Position error (m, log scale)")
    ax_pos_err.set_title("Position error over time")
    _clean_axes(ax_pos_err)
    ax_pos_err.legend(loc="upper left", fontsize=8, frameon=False)

    ax_att_err = fig.add_subplot(1, 3, 3)
    for i, (name, result) in enumerate(results.items()):
        color = _SERIES_COLORS[i % len(_SERIES_COLORS)]
        errors = np.maximum(_attitude_errors_deg(result), 1e-9)
        ax_att_err.plot(t, errors, color=color, linewidth=2, label=name)

    ax_att_err.set_yscale("log")
    ax_att_err.set_xlabel("Time (s)")
    ax_att_err.set_ylabel("Attitude error (degrees, log scale)")
    ax_att_err.set_title("Attitude error over time")
    _clean_axes(ax_att_err)
    ax_att_err.legend(loc="upper left", fontsize=8, frameon=False)

    fig.tight_layout(rect=(0, 0, 1, 0.94) if title else None)

    return fig


def _clean_axes(ax) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, which="both", linewidth=0.4, alpha=0.4)
