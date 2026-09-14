"""Pure quaternion kinematics: angular-rate integration primitives.

Ported from the root-level ``Functions.py``. ``Functions.q_update`` was not
ported - it is never called anywhere in the scripts it was written for, and
attempting to run it raises ``TypeError`` (it divides a ``Quaternion`` by a
scalar via ``/``, an operator ``Quaternion`` does not implement), so there is
no validated behaviour to preserve.

"""

import numpy as np

from .quaternion import Quaternion


def omega_matrix(w) -> np.ndarray:
    """
    Build the 4x4 matrix such that ``0.5 * omega_matrix(w) @ q`` is the time
    derivative of the quaternion ``q`` for a body rotating with angular
    velocity ``w`` (rad/s).

    Args:
        * w {``array-like``} -- Angular velocity 3-vector (rad/s).

    Returns:
        * {``np.ndarray``} -- The 4x4 multiplication matrix.

    """
    w = np.asarray(w, dtype=float)
    if w.shape != (3,):
        raise ValueError(f"w must be a 3-vector, got shape {w.shape}")

    return np.array(
        [
            [0, -w[0], -w[1], -w[2]],
            [w[0], 0, w[2], -w[1]],
            [w[1], -w[2], 0, w[0]],
            [w[2], w[1], -w[0], 0],
        ]
    )


def rk4_attitude_step(qi: Quaternion, w1, w2, dt: float) -> Quaternion:
    """
    Integrate attitude over one time step using 4th order Runge-Kutta.

    Args:
        * qi {``Quaternion``} -- Attitude quaternion at the start of the step.
        * w1 {``array-like``} -- Angular velocity 3-vector (degrees/s) at the
          start of the step.
        * w2 {``array-like``} -- Angular velocity 3-vector (degrees/s) at the
          end of the step.
        * dt {``float``} -- Time step (s).

    Returns:
        * {``Quaternion``} -- Attitude quaternion at the end of the step.

    """
    qt = np.array([qi.w, qi.x, qi.y, qi.z])
    w11 = np.radians(np.asarray(w1, dtype=float))
    w22 = np.radians(np.asarray(w2, dtype=float))

    q1 = qt
    k1 = 0.5 * omega_matrix(w11) @ q1
    q2 = qt + dt * 0.5 * k1
    k2 = 0.5 * omega_matrix(0.5 * (w11 + w22)) @ q2
    q3 = qt + dt * 0.5 * k2
    k3 = 0.5 * omega_matrix(0.5 * (w11 + w22)) @ q3
    q4 = qt + dt * k3
    k4 = 0.5 * omega_matrix(w22) @ q4

    qf = qt + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    qf = qf / np.linalg.norm(qf)

    return Quaternion(*qf)
