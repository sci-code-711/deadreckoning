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

    Segment durations are deliberately irregular (not round numbers) so
    their boundaries don't land exactly on the sample grid at common test
    rates (10-1000 Hz) - a real sensor's fixed sample clock has no reason
    to line up with a real discontinuity's timing, and testing with
    boundaries that coincidentally do line up gives a misleadingly easy
    case. The acceleration/turn-rate magnitudes are chosen to exactly
    compensate the non-round durations, so the same known velocity
    milestones (6, 3, 0 m/s; a 90 degree turn) still hold exactly.

    """
    return SyntheticTrajectory(
        [
            MotionSegment(duration=3.0402, accel_nav=[1.9735543714229327, 0.0, 0.0]),  # 0->6 m/s
            MotionSegment(duration=4.0532, accel_nav=[0.0, 0.0, 0.0]),  # cruise at 6 m/s
            MotionSegment(duration=2.9592, accel_nav=[-1.013787510137875, 0.0, 0.0]),  # 6->3 m/s
            CoordinatedTurnSegment(duration=2.9775, turn_rate_deg=30.22670025188917),  # 90 deg
            MotionSegment(duration=3.0406, accel_nav=[0.0, -0.9866473722291653, 0.0]),  # 3->0 m/s
        ],
        gravity_magnitude=_G,
    )


def full_composite() -> SyntheticTrajectory:
    """
    A stress-test maneuver mixing simultaneous multi-axis attitude change
    (roll/pitch/yaw together) with translation and a coordinated turn, so
    attitude and position both evolve non-trivially and off-axis.

    Segment durations are deliberately irregular (not round numbers), for
    the same reason as :func:`composite_sequence` - so boundaries don't
    coincidentally land on the sample grid at common test rates.

    """
    return SyntheticTrajectory(
        [
            MotionSegment(
                duration=2.0176, accel_nav=[1.0, 0.0, 0.0], angular_velocity_body=[15.0, 10.0, 5.0]
            ),
            MotionSegment(
                duration=2.0589,
                accel_nav=[0.0, 1.0, 0.5],
                angular_velocity_body=[-10.0, 15.0, -5.0],
            ),
            CoordinatedTurnSegment(duration=1.9816, turn_rate_deg=20.0),
            MotionSegment(
                duration=2.0337,
                accel_nav=[-0.5, -0.5, 0.0],
                angular_velocity_body=[5.0, -5.0, 10.0],
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
