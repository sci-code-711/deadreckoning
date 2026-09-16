import numpy as np
import pytest

from deadrec.interpolation import (
    CentredCubicHermiteInterpolator,
    TwoPointLinearInterpolator,
    ZeroOrderHoldInterpolator,
)
from deadrec.samples import ImuSample


def _window():
    return [
        ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[10.0, 0.0, -20.0]),
        ImuSample(t=0.1, accel=[0.0, 0.0, 9.8], gyro=[30.0, 40.0, -60.0]),
    ]


def test_two_point_linear_is_causal_with_no_extra_context():
    interp = TwoPointLinearInterpolator()

    assert interp.context_before == 0
    assert interp.context_after == 0
    assert interp.causal is True


def test_two_point_linear_endpoints_match_converted_gyro_readings():
    window = _window()
    omega = TwoPointLinearInterpolator().build(window, step_pos=1)

    assert np.allclose(omega(window[0].t), np.radians(window[0].gyro))
    assert np.allclose(omega(window[1].t), np.radians(window[1].gyro))


def test_two_point_linear_blend_is_exact():
    window = _window()
    omega = TwoPointLinearInterpolator().build(window, step_pos=1)

    w0 = np.radians(window[0].gyro)
    w1 = np.radians(window[1].gyro)
    t_mid = 0.5 * (window[0].t + window[1].t)

    assert np.allclose(omega(t_mid), 0.5 * (w0 + w1))

    frac = 0.3
    t = window[0].t + frac * (window[1].t - window[0].t)
    assert np.allclose(omega(t), w0 + frac * (w1 - w0))


def test_zero_order_hold_is_causal_with_no_extra_context():
    interp = ZeroOrderHoldInterpolator()

    assert interp.context_before == 0
    assert interp.context_after == 0
    assert interp.causal is True


def test_zero_order_hold_endpoint_matches_converted_gyro_reading():
    window = _window()
    omega = ZeroOrderHoldInterpolator().build(window, step_pos=1)

    assert np.allclose(omega(window[0].t), np.radians(window[0].gyro))


def test_zero_order_hold_is_constant_over_the_step():
    window = _window()
    omega = ZeroOrderHoldInterpolator().build(window, step_pos=1)

    w0 = np.radians(window[0].gyro)
    assert np.allclose(omega(window[0].t), w0)
    assert np.allclose(omega(0.5 * (window[0].t + window[1].t)), w0)
    assert np.allclose(omega(window[1].t), w0)


@pytest.mark.parametrize("cls", [TwoPointLinearInterpolator, ZeroOrderHoldInterpolator])
def test_build_works_mid_window_with_extra_context_samples(cls):
    # step_pos need not be 1 / len(window)-1 - build() should only care
    # about window[step_pos - 1] and window[step_pos].
    window = [
        ImuSample(t=-0.1, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0]),
        *_window(),
        ImuSample(t=0.2, accel=[0.0, 0.0, 9.8], gyro=[99.0, 99.0, 99.0]),
    ]
    omega = cls().build(window, step_pos=2)

    assert np.allclose(omega(window[1].t), np.radians(window[1].gyro))


def test_centred_cubic_hermite_is_non_causal():
    interp = CentredCubicHermiteInterpolator()

    assert interp.context_before == 1
    assert interp.context_after == 1
    assert interp.causal is False


def _hermite_window():
    return [
        ImuSample(t=0.0, accel=[0.0, 0.0, 9.8], gyro=[0.0, 0.0, 0.0]),
        ImuSample(t=1.0, accel=[0.0, 0.0, 9.8], gyro=[10.0, 0.0, 0.0]),
        ImuSample(t=2.0, accel=[0.0, 0.0, 9.8], gyro=[20.0, 0.0, 0.0]),
        ImuSample(t=3.0, accel=[0.0, 0.0, 9.8], gyro=[40.0, 0.0, 0.0]),
    ]


def test_centred_cubic_hermite_endpoints_match_converted_gyro_readings():
    window = _hermite_window()
    omega = CentredCubicHermiteInterpolator().build(window, step_pos=2)

    assert np.allclose(omega(window[1].t), np.radians(window[1].gyro))
    assert np.allclose(omega(window[2].t), np.radians(window[2].gyro))


def test_centred_cubic_hermite_matches_hand_derived_value():
    window = _hermite_window()
    omega = CentredCubicHermiteInterpolator().build(window, step_pos=2)

    # Catmull-Rom tangents (deg/s): m0 = (20 - 0) / (2 - 0) = 10,
    # m1 = (40 - 10) / (3 - 1) = 15. span = 1, s = 0.5 gives basis weights
    # h00 = 0.5, h10 = 0.125, h01 = 0.5, h11 = -0.125, so:
    # value = 0.5*10 + 0.125*1*10 + 0.5*20 - 0.125*1*15 = 14.375 deg/s.
    expected = np.radians([14.375, 0.0, 0.0])

    assert np.allclose(omega(1.5), expected)


def test_centred_cubic_hermite_degrades_gracefully_near_sequence_start():
    # Only 3 samples: step_pos=1 has no context before it (index -1), but
    # context after is available - shouldn't raise, and endpoints should
    # still match exactly.
    window = _hermite_window()[:3]
    omega = CentredCubicHermiteInterpolator().build(window, step_pos=1)

    assert np.allclose(omega(window[0].t), np.radians(window[0].gyro))
    assert np.allclose(omega(window[1].t), np.radians(window[1].gyro))
    assert np.all(np.isfinite(omega(0.5)))


def test_centred_cubic_hermite_degrades_gracefully_near_sequence_end():
    window = _hermite_window()[1:]
    # window = [t=1,t=2,t=3]; step_pos=2 -> start=window[1](t=2), end=window[2](t=3),
    # no context after (index 3 out of bounds), context before available.
    omega = CentredCubicHermiteInterpolator().build(window, step_pos=2)

    assert np.allclose(omega(window[1].t), np.radians(window[1].gyro))
    assert np.allclose(omega(window[2].t), np.radians(window[2].gyro))
    assert np.all(np.isfinite(omega(2.5)))


def test_centred_cubic_hermite_falls_back_to_linear_with_no_context_available():
    window = _hermite_window()[1:3]  # just the 2 step endpoints, no context either side
    omega = CentredCubicHermiteInterpolator().build(window, step_pos=1)

    w0 = np.radians(window[0].gyro)
    w1 = np.radians(window[1].gyro)
    assert np.allclose(omega(0.5 * (window[0].t + window[1].t)), 0.5 * (w0 + w1))
