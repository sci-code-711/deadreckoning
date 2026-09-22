import numpy as np
import pytest

from deadrec.curves import PolynomialCurve


def test_rejects_non_2d_coefficients():
    with pytest.raises(ValueError):
        PolynomialCurve([1.0, 2.0, 3.0])


def test_rejects_wrong_second_dimension():
    with pytest.raises(ValueError):
        PolynomialCurve([[1.0, 2.0], [3.0, 4.0]])


def test_quadratic_single_axis():
    # x(t) = t^2, y(t) = z(t) = 0.
    curve = PolynomialCurve([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])

    for t in [0.0, 2.0, -3.0]:
        assert np.allclose(curve.value(t), [t**2, 0.0, 0.0])
        assert np.allclose(curve.velocity(t), [2 * t, 0.0, 0.0])
        assert np.allclose(curve.acceleration(t), [2.0, 0.0, 0.0])


def test_cubic_single_axis():
    # x(t) = t^3 - 2t + 1.
    curve = PolynomialCurve([[1.0, 0.0, 0.0], [-2.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])

    for t in [0.0, 1.0, -2.0, 3.5]:
        assert np.allclose(curve.value(t), [t**3 - 2 * t + 1, 0.0, 0.0])
        assert np.allclose(curve.velocity(t), [3 * t**2 - 2, 0.0, 0.0])
        assert np.allclose(curve.acceleration(t), [6 * t, 0.0, 0.0])


def test_independent_polynomials_per_axis():
    # x(t) = t, y(t) = t^2, z(t) = t^3.
    coefficients = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    curve = PolynomialCurve(coefficients)

    t = 2.0
    assert np.allclose(curve.value(t), [t, t**2, t**3])
    assert np.allclose(curve.velocity(t), [1.0, 2 * t, 3 * t**2])
    assert np.allclose(curve.acceleration(t), [0.0, 2.0, 6 * t])


def _central_difference_velocity(curve, t, h=1e-4):
    return (curve.value(t + h) - curve.value(t - h)) / (2 * h)


def _central_difference_acceleration(curve, t, h=1e-4):
    return (curve.value(t + h) - 2 * curve.value(t) + curve.value(t - h)) / h**2


def test_derivatives_match_independent_central_difference():
    rng = np.random.default_rng(0)

    for _ in range(10):
        degree = rng.integers(1, 6)
        coefficients = rng.uniform(-5.0, 5.0, size=(degree + 1, 3))
        curve = PolynomialCurve(coefficients)

        for t in rng.uniform(-3.0, 3.0, size=5):
            assert np.allclose(curve.velocity(t), _central_difference_velocity(curve, t), atol=1e-3)
            assert np.allclose(
                curve.acceleration(t), _central_difference_acceleration(curve, t), atol=1e-2
            )
