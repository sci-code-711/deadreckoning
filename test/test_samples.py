import numpy as np
import pytest

from deadrec.quaternion import Quaternion
from deadrec.samples import ImuSample, TrajectoryState


def test_imu_sample_init():
    sample = ImuSample(t=1.5, accel=[0.1, 0.2, 9.8], gyro=(1.0, -2.0, 3.0))

    assert sample.t == 1.5
    assert np.array_equal(sample.accel, np.array([0.1, 0.2, 9.8]))
    assert np.array_equal(sample.gyro, np.array([1.0, -2.0, 3.0]))


def test_imu_sample_rejects_wrong_shape():
    with pytest.raises(ValueError):
        ImuSample(t=0.0, accel=[0.1, 0.2], gyro=[0.0, 0.0, 0.0])

    with pytest.raises(ValueError):
        ImuSample(t=0.0, accel=[0.1, 0.2, 0.3], gyro=[0.0, 0.0, 0.0, 0.0])


def test_trajectory_state_init():
    state = TrajectoryState(
        t=2.0,
        attitude=Quaternion(1, 0, 0, 0),
        accel_nav=[0.0, 0.0, 0.0],
        velocity=[1.0, 0.0, 0.0],
        position=[0.0, 1.0, 0.0],
        euler=(0.0, 0.0, 0.0),
    )

    assert state.t == 2.0
    assert state.attitude == Quaternion(1, 0, 0, 0)
    assert np.array_equal(state.accel_nav, np.array([0.0, 0.0, 0.0]))
    assert np.array_equal(state.velocity, np.array([1.0, 0.0, 0.0]))
    assert np.array_equal(state.position, np.array([0.0, 1.0, 0.0]))
    assert state.euler == (0.0, 0.0, 0.0)


def test_trajectory_state_default_euler():
    state = TrajectoryState(
        t=0.0,
        attitude=Quaternion(1, 0, 0, 0),
        accel_nav=[0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0],
        position=[0.0, 0.0, 0.0],
    )

    assert state.euler == (0.0, 0.0, 0.0)


def test_trajectory_state_rejects_non_quaternion_attitude():
    with pytest.raises(TypeError):
        TrajectoryState(
            t=0.0,
            attitude=(1, 0, 0, 0),
            accel_nav=[0.0, 0.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            position=[0.0, 0.0, 0.0],
        )


def test_trajectory_state_rejects_wrong_shape():
    with pytest.raises(ValueError):
        TrajectoryState(
            t=0.0,
            attitude=Quaternion(1, 0, 0, 0),
            accel_nav=[0.0, 0.0],
            velocity=[0.0, 0.0, 0.0],
            position=[0.0, 0.0, 0.0],
        )
