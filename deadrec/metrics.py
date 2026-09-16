"""Accuracy metrics for comparing a reconstructed trajectory against a
reference.

Extracted as reusable building blocks (rather than test-only helpers) so a
future benchmarking tool (see the deferred `deadrec benchmark` phase in #41)
can import them alongside the test suite.
"""

import numpy as np

from .quaternion import Quaternion
from .samples import TrajectoryState


def position_errors(states: list[TrajectoryState], reference_positions) -> np.ndarray:
    """
    Per-step Euclidean distance between reconstructed and reference
    positions.

    Args:
        * states {``list[TrajectoryState]``} -- Reconstructed trajectory,
          e.g. from :meth:`deadrec.dead_reckoning.DeadReckoner.run`.
        * reference_positions {``array-like``} -- (N, 3) reference/ground-truth
          positions, one per state, in the same order.

    Returns:
        * {``np.ndarray``} -- (N,) Euclidean position error at each step.

    """
    reconstructed = np.array([state.position for state in states])
    reference = np.asarray(reference_positions, dtype=float)

    return np.linalg.norm(reconstructed - reference, axis=1)


def attitude_angle_error_deg(a: Quaternion, b: Quaternion) -> float:
    """
    Angular difference between two attitude quaternions, in degrees - the
    rotation angle of ``a``'s attitude relative to ``b``'s.

    Args:
        * a {``Quaternion``} -- The first attitude.
        * b {``Quaternion``} -- The second attitude.

    Returns:
        * {``float``} -- The angle between the two attitudes, in ``[0, 180]``.

    """
    dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
    # Quaternions q and -q represent the same attitude, and a small amount
    # of floating-point drift can push |dot| fractionally above 1.
    dot = min(1.0, abs(dot))

    return float(np.degrees(2 * np.arccos(dot)))
