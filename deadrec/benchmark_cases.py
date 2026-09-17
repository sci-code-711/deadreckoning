"""The 6 canonical synthetic benchmark trajectories.

Each function below builds a :class:`deadrec.synthetic.SyntheticTrajectory`
covering one key physical case a dead-reckoning/EKF algorithm should be able
to handle, from the simplest (stationary) up to a full 3D composite
maneuver. :data:`BENCHMARK_CASES` collects them all for easy iteration, e.g.
when benchmarking multiple reckoner classes across every case.
"""

from collections.abc import Callable

from .synthetic import CoordinatedTurnSegment, MotionSegment, SyntheticTrajectory

_G = 9.80665


def stationary() -> SyntheticTrajectory:
    """
    The sensor sits still for 5 seconds: zero acceleration, zero rotation.
    The baseline case - a correct reckoner should show ~zero drift.

    """
    return SyntheticTrajectory([MotionSegment(duration=5.0)], gravity_magnitude=_G)


def constant_rotation() -> SyntheticTrajectory:
    """
    The sensor spins in place about a single (vertical) axis at a steady 30
    deg/s for 6 seconds, with no translation. Isolates pure gyroscope
    integration / attitude tracking from any acceleration.

    """
    return SyntheticTrajectory(
        [MotionSegment(duration=6.0, angular_velocity_body=[0.0, 0.0, 30.0])],
        gravity_magnitude=_G,
    )


def constant_linear_acceleration() -> SyntheticTrajectory:
    """
    The sensor accelerates in a straight line at a steady 2 m/s^2 for 5
    seconds, with attitude fixed. Isolates pure accelerometer integration to
    velocity/position from any rotation.

    """
    return SyntheticTrajectory(
        [MotionSegment(duration=5.0, accel_nav=[2.0, 0.0, 0.0])], gravity_magnitude=_G
    )


def circular_motion() -> SyntheticTrajectory:
    """
    The sensor travels at a constant 5 m/s around a level circular arc,
    turning at a steady 20 deg/s for 9 seconds (a full 180 degrees).
    Exercises the coupling between rotation and acceleration: centripetal
    acceleration rotates in the navigation frame as heading changes.

    """
    return SyntheticTrajectory(
        [CoordinatedTurnSegment(duration=9.0, turn_rate_deg=20.0)],
        initial_velocity=[5.0, 0.0, 0.0],
        gravity_magnitude=_G,
    )


def composite_sequence() -> SyntheticTrajectory:
    """
    A simple "driving route": accelerate from rest to 6 m/s, cruise,
    decelerate to 3 m/s, take a level 90-degree turn at that speed, then
    decelerate to a stop. A realistic multi-phase motion exercising every
    primitive end to end, with a fully known final state (back at rest).

    """
    return SyntheticTrajectory(
        [
            MotionSegment(duration=3.0, accel_nav=[2.0, 0.0, 0.0]),  # 0 -> 6 m/s
            MotionSegment(duration=4.0, accel_nav=[0.0, 0.0, 0.0]),  # cruise at 6 m/s
            MotionSegment(duration=3.0, accel_nav=[-1.0, 0.0, 0.0]),  # 6 -> 3 m/s
            CoordinatedTurnSegment(duration=3.0, turn_rate_deg=30.0),  # 90 degree turn
            MotionSegment(duration=3.0, accel_nav=[0.0, -1.0, 0.0]),  # 3 -> 0 m/s
        ],
        gravity_magnitude=_G,
    )


def full_composite() -> SyntheticTrajectory:
    """
    A stress-test maneuver mixing simultaneous multi-axis attitude change
    (roll/pitch/yaw together) with translation and a coordinated turn, so
    attitude and position both evolve non-trivially and off-axis.

    """
    return SyntheticTrajectory(
        [
            MotionSegment(
                duration=2.0, accel_nav=[1.0, 0.0, 0.0], angular_velocity_body=[15.0, 10.0, 5.0]
            ),
            MotionSegment(
                duration=2.0, accel_nav=[0.0, 1.0, 0.5], angular_velocity_body=[-10.0, 15.0, -5.0]
            ),
            CoordinatedTurnSegment(duration=2.0, turn_rate_deg=20.0),
            MotionSegment(
                duration=2.0, accel_nav=[-0.5, -0.5, 0.0], angular_velocity_body=[5.0, -5.0, 10.0]
            ),
        ],
        gravity_magnitude=_G,
    )


BENCHMARK_CASES: dict[str, Callable[[], SyntheticTrajectory]] = {
    "stationary": stationary,
    "constant_rotation": constant_rotation,
    "constant_linear_acceleration": constant_linear_acceleration,
    "circular_motion": circular_motion,
    "composite_sequence": composite_sequence,
    "full_composite": full_composite,
}
