import numpy as np
import pytest

from deadrec.quintic_hermite_spline import QuinticHermitePositionSpline


def test_requires_at_least_two_waypoints():
    with pytest.raises(ValueError):
        QuinticHermitePositionSpline([(0.0, [0.0, 0.0, 0.0])])


def test_requires_strictly_increasing_times():
    with pytest.raises(ValueError):
        QuinticHermitePositionSpline([(0.0, [0.0, 0.0, 0.0]), (0.0, [1.0, 0.0, 0.0])])

    with pytest.raises(ValueError):
        QuinticHermitePositionSpline([(1.0, [0.0, 0.0, 0.0]), (0.0, [1.0, 0.0, 0.0])])


def test_rejects_time_outside_its_range():
    spline = QuinticHermitePositionSpline([(0.0, [0.0, 0.0, 0.0]), (1.0, [1.0, 0.0, 0.0])])

    with pytest.raises(ValueError):
        spline.value(-0.1)
    with pytest.raises(ValueError):
        spline.velocity(1.1)
    with pytest.raises(ValueError):
        spline.acceleration(1.1)


def test_passes_through_every_waypoint_position_exactly():
    waypoints = [
        (0.0, [0.0, 0.0, 0.0]),
        (1.0, [1.0, 2.0, -1.0]),
        (2.5, [0.5, 3.0, 0.0]),
        (4.0, [2.0, 1.0, 1.0]),
    ]
    spline = QuinticHermitePositionSpline(waypoints)

    for t, p in waypoints:
        assert np.allclose(spline.value(t), p, atol=1e-9)


def test_explicit_waypoint_accelerations_are_honoured_exactly():
    waypoints = [
        (0.0, [0.0, 0.0, 0.0], [1.0, -2.0, 0.5]),
        (1.0, [1.0, 0.0, 0.0]),  # defaults to zero acceleration
        (2.0, [1.0, 1.0, 0.0], [0.0, 3.0, -1.0]),
    ]
    spline = QuinticHermitePositionSpline(waypoints)

    assert np.allclose(spline.acceleration(0.0), [1.0, -2.0, 0.5], atol=1e-9)
    assert np.allclose(spline.acceleration(1.0), [0.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(spline.acceleration(2.0), [0.0, 3.0, -1.0], atol=1e-9)


def test_value_velocity_and_acceleration_are_continuous_across_segment_boundaries():
    waypoints = [
        (0.0, [0.0, 0.0, 0.0]),
        (1.0, [1.0, -1.0, 2.0], [0.5, 0.0, -0.5]),
        (1.7, [2.0, 1.0, 1.0]),
        (3.2, [1.0, 2.0, -1.0], [-1.0, 1.0, 0.0]),
    ]
    spline = QuinticHermitePositionSpline(waypoints)

    boundary_times = [t for t, *_ in waypoints[1:-1]]
    for i, t in enumerate(boundary_times):
        left, right = spline._segments[i], spline._segments[i + 1]

        assert np.allclose(left.value(t), right.value(t), atol=1e-9)
        assert np.allclose(left.velocity(t), right.velocity(t), atol=1e-9)
        assert np.allclose(left.acceleration(t), right.acceleration(t), atol=1e-9)


def test_degenerate_collinear_evenly_spaced_waypoints_reduce_to_a_straight_line():
    velocity = np.array([2.0, -1.0, 0.5])
    waypoints = [(float(t), velocity * t) for t in range(5)]
    spline = QuinticHermitePositionSpline(waypoints)

    for t in [0.0, 0.3, 1.5, 2.7, 3.9, 4.0]:
        assert np.allclose(spline.value(t), velocity * t, atol=1e-8)
        assert np.allclose(spline.velocity(t), velocity, atol=1e-8)
        assert np.allclose(spline.acceleration(t), [0.0, 0.0, 0.0], atol=1e-8)


def _central_difference_velocity(spline, t, h=1e-4):
    return (spline.value(t + h) - spline.value(t - h)) / (2 * h)


def _central_difference_acceleration(spline, t, h=1e-4):
    return (spline.value(t + h) - 2 * spline.value(t) + spline.value(t - h)) / h**2


def test_derivatives_match_independent_central_difference():
    waypoints = [
        (0.0, [0.0, 0.0, 0.0]),
        (1.0, [1.0, 2.0, -1.0], [2.0, 0.0, 1.0]),
        (2.2, [0.0, 3.0, 1.0]),
        (3.5, [-1.0, 1.0, 2.0], [0.0, -1.0, 0.0]),
    ]
    spline = QuinticHermitePositionSpline(waypoints)

    for t in [0.5, 1.6, 2.8]:
        assert np.allclose(spline.velocity(t), _central_difference_velocity(spline, t), atol=1e-3)
        assert np.allclose(
            spline.acceleration(t), _central_difference_acceleration(spline, t), atol=1e-2
        )
