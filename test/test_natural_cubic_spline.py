import numpy as np
import pytest

from deadrec.natural_cubic_spline import NaturalCubicPositionSpline


def test_rejects_fewer_than_two_waypoints():
    with pytest.raises(ValueError):
        NaturalCubicPositionSpline([(0.0, [0, 0, 0])])


def test_rejects_non_increasing_times():
    with pytest.raises(ValueError):
        NaturalCubicPositionSpline([(0.0, [0, 0, 0]), (0.0, [1, 0, 0])])
    with pytest.raises(ValueError):
        NaturalCubicPositionSpline([(1.0, [0, 0, 0]), (0.0, [1, 0, 0])])


def test_rejects_wrong_shaped_position():
    with pytest.raises(ValueError):
        NaturalCubicPositionSpline([(0.0, [0, 0]), (1.0, [1, 1, 1])])


def test_interpolates_every_waypoint_exactly():
    waypoints = [
        (0.0, [0.0, 0.0, 0.0]),
        (1.0, [1.0, 2.0, -1.0]),
        (2.5, [0.5, -1.0, 3.0]),
        (4.0, [2.0, 0.0, 0.0]),
        (5.0, [-1.0, 1.0, 1.0]),
    ]
    spline = NaturalCubicPositionSpline(waypoints)

    for t, p in waypoints:
        assert np.allclose(spline.value(t), p)


def test_continuity_across_segment_boundaries():
    waypoints = [
        (0.0, [0.0, 0.0, 0.0]),
        (1.0, [1.0, 2.0, -1.0]),
        (2.5, [0.5, -1.0, 3.0]),
        (4.0, [2.0, 0.0, 0.0]),
    ]
    spline = NaturalCubicPositionSpline(waypoints)
    eps = 1e-6

    for t, _ in waypoints[1:-1]:
        assert np.allclose(spline.value(t - eps), spline.value(t + eps), atol=1e-5)
        assert np.allclose(spline.velocity(t - eps), spline.velocity(t + eps), atol=1e-4)
        assert np.allclose(spline.acceleration(t - eps), spline.acceleration(t + eps), atol=1e-2)


def test_matches_hand_derived_three_point_example():
    # Textbook natural cubic spline through (0,0), (1,1), (2,0) on the x
    # axis (y/z held at 0 throughout, trivially). Hand-derived (see the
    # implementation plan): M_1 = -3, giving
    #   S_0(tau) = -0.5*tau^3 + 1.5*tau           for t in [0, 1], tau=t
    #   S_1(tau) = -0.5*(1-tau)^3 + 1.5*(1-tau)   for t in [1, 2], tau=t-1
    # independent of accel_to_nav_frame/curves.py's own machinery - this
    # is arithmetic done by hand, not re-derived from the implementation.
    spline = NaturalCubicPositionSpline(
        [(0.0, [0.0, 0.0, 0.0]), (1.0, [1.0, 0.0, 0.0]), (2.0, [0.0, 0.0, 0.0])]
    )

    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        tau = t
        expected_x = -0.5 * tau**3 + 1.5 * tau
        expected_vx = -1.5 * tau**2 + 1.5
        expected_ax = -3 * tau
        assert spline.value(t)[0] == pytest.approx(expected_x, abs=1e-9)
        assert spline.velocity(t)[0] == pytest.approx(expected_vx, abs=1e-9)
        assert spline.acceleration(t)[0] == pytest.approx(expected_ax, abs=1e-9)

    for t in [1.0, 1.25, 1.5, 1.75, 2.0]:
        tau = t - 1.0
        expected_x = -0.5 * (1 - tau) ** 3 + 1.5 * (1 - tau)
        expected_vx = 1.5 * (1 - tau) ** 2 - 1.5
        expected_ax = -3 * (1 - tau)
        assert spline.value(t)[0] == pytest.approx(expected_x, abs=1e-9)
        assert spline.velocity(t)[0] == pytest.approx(expected_vx, abs=1e-9)
        assert spline.acceleration(t)[0] == pytest.approx(expected_ax, abs=1e-9)


def test_collinear_evenly_spaced_waypoints_reduce_to_a_straight_line():
    velocity = np.array([2.0, 1.0, -1.0])
    waypoints = [(float(i), (velocity * i).tolist()) for i in range(5)]
    spline = NaturalCubicPositionSpline(waypoints)

    for t in np.linspace(0.0, 4.0, 21):
        assert np.allclose(spline.value(t), velocity * t, atol=1e-9)
        assert np.allclose(spline.velocity(t), velocity, atol=1e-9)
        assert np.allclose(spline.acceleration(t), [0.0, 0.0, 0.0], atol=1e-9)


def test_two_waypoints_is_a_straight_line():
    spline = NaturalCubicPositionSpline([(0.0, [0.0, 0.0, 0.0]), (2.0, [4.0, -2.0, 0.0])])

    assert np.allclose(spline.value(1.0), [2.0, -1.0, 0.0])
    assert np.allclose(spline.velocity(1.0), [2.0, -1.0, 0.0])
    assert np.allclose(spline.acceleration(1.0), [0.0, 0.0, 0.0])


def _central_difference_velocity(spline, t, h=1e-4):
    return (spline.value(t + h) - spline.value(t - h)) / (2 * h)


def _central_difference_acceleration(spline, t, h=1e-4):
    return (spline.value(t + h) - 2 * spline.value(t) + spline.value(t - h)) / h**2


def test_derivatives_match_independent_central_difference():
    rng = np.random.default_rng(1)
    times = np.array([0.0, 0.8, 2.1, 2.9, 4.5, 6.0])
    positions = rng.uniform(-5.0, 5.0, size=(len(times), 3))
    spline = NaturalCubicPositionSpline(list(zip(times, positions)))

    for t in np.linspace(0.2, 5.8, 15):
        assert np.allclose(spline.velocity(t), _central_difference_velocity(spline, t), atol=1e-3)
        assert np.allclose(
            spline.acceleration(t), _central_difference_acceleration(spline, t), atol=1e-2
        )


def test_rejects_time_outside_range():
    spline = NaturalCubicPositionSpline([(0.0, [0.0, 0.0, 0.0]), (1.0, [1.0, 1.0, 1.0])])

    with pytest.raises(ValueError):
        spline.value(-0.1)
    with pytest.raises(ValueError):
        spline.value(1.1)
