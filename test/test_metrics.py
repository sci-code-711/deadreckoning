import numpy as np
import pytest

from deadrec.metrics import attitude_angle_error_deg, position_errors
from deadrec.quaternion import Quaternion
from deadrec.samples import TrajectoryState


def _state(position):
    return TrajectoryState(
        t=0.0,
        attitude=Quaternion(1, 0, 0, 0),
        accel_nav=[0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0],
        position=position,
    )


def test_position_errors_is_zero_for_identical_trajectories():
    states = [_state([0.0, 0.0, 0.0]), _state([1.0, 2.0, 3.0])]
    reference = [[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]

    assert np.allclose(position_errors(states, reference), [0.0, 0.0])


def test_position_errors_computes_euclidean_distance_per_step():
    states = [_state([0.0, 0.0, 0.0]), _state([3.0, 0.0, 0.0])]
    reference = [[0.0, 4.0, 0.0], [0.0, 0.0, 0.0]]

    # step 0: |[0,0,0] - [0,4,0]| = 4; step 1: |[3,0,0] - [0,0,0]| = 3.
    assert np.allclose(position_errors(states, reference), [4.0, 3.0])


def test_attitude_angle_error_deg_is_zero_for_identical_attitudes():
    q = Quaternion.from_eul_angles(0.3, -0.2, 0.5)

    assert attitude_angle_error_deg(q, q) == pytest.approx(0.0, abs=1e-9)


def test_attitude_angle_error_deg_is_sign_invariant():
    q = Quaternion.from_eul_angles(0.3, -0.2, 0.5)

    assert attitude_angle_error_deg(q, -q) == pytest.approx(0.0, abs=1e-9)


def test_attitude_angle_error_deg_matches_known_rotation():
    identity = Quaternion(1, 0, 0, 0)
    # from_eul_angles takes half-angles (see test_kinematics.py's
    # `wz_rad * dt / 2` usage), so radians(15) here is a 30-degree rotation.
    rotated = Quaternion.from_eul_angles(0, 0, np.radians(15))

    assert attitude_angle_error_deg(identity, rotated) == pytest.approx(30.0, abs=1e-6)


def test_attitude_angle_error_deg_is_symmetric():
    a = Quaternion.from_eul_angles(0.1, 0.2, 0.3)
    b = Quaternion.from_eul_angles(-0.2, 0.4, 0.1)

    assert attitude_angle_error_deg(a, b) == pytest.approx(attitude_angle_error_deg(b, a))
