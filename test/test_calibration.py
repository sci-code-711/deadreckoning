import numpy as np
import pytest

from deadrec.calibration import (
    CalibrationCoefficients,
    apply_calibration,
    estimate_gyro_bias,
)
from deadrec.samples import ImuSample

# Real calibration coefficients for one physical IMU, reused here so the
# regression test exercises real numbers rather than arbitrary ones.
_A_COEFF = np.array(
    [
        6.08032845e-03,
        -6.04269967e-03,
        -4.73899395e-03,
        9.77255708e00,
        9.82729876e00,
        9.81958972e00,
        -9.78688601e-03,
        -5.66163490e-04,
        -1.29535798e-02,
    ]
)
_G_COEFF = np.array(
    [
        0.03702091,
        -0.02547652,
        0.01486421,
        0.00508759,
        0.00590621,
        -0.01795366,
        1.13553109,
        1.16152047,
        1.13504326,
    ]
)


def test_from_raw_coefficients_slices_match_calibration_py():
    coeffs = CalibrationCoefficients.from_raw_coefficients(_A_COEFF, _G_COEFF, gyro_bias=[0, 0, 0])

    assert np.array_equal(coeffs.accel_scale, _A_COEFF[3:6])
    assert np.array_equal(coeffs.accel_bias, _A_COEFF[6:9])
    assert np.array_equal(coeffs.gyro_scale, _G_COEFF[6:9])

    expected_accel_misalignment = np.array(
        [
            [1, -_A_COEFF[0], _A_COEFF[1]],
            [0, 1, -_A_COEFF[2]],
            [0, 0, 1],
        ]
    )
    assert np.array_equal(coeffs.accel_misalignment, expected_accel_misalignment)

    expected_gyro_misalignment = np.array(
        [
            [1, -_G_COEFF[0], _G_COEFF[1]],
            [_G_COEFF[2], 1, -_G_COEFF[3]],
            [-_G_COEFF[4], _G_COEFF[5], 1],
        ]
    )
    assert np.array_equal(coeffs.gyro_misalignment, expected_gyro_misalignment)


def test_from_raw_coefficients_rejects_wrong_shape():
    with pytest.raises(ValueError):
        CalibrationCoefficients.from_raw_coefficients([1, 2, 3], _G_COEFF, gyro_bias=[0, 0, 0])

    with pytest.raises(ValueError):
        CalibrationCoefficients.from_raw_coefficients(_A_COEFF, [1, 2, 3], gyro_bias=[0, 0, 0])


def test_apply_calibration_identity_coefficients_leave_sample_unchanged():
    coeffs = CalibrationCoefficients(
        accel_misalignment=np.eye(3),
        accel_scale=np.array([1.0, 1.0, 1.0]),
        accel_bias=np.array([0.0, 0.0, 0.0]),
        gyro_misalignment=np.eye(3),
        gyro_scale=np.array([1.0, 1.0, 1.0]),
        gyro_bias=np.array([0.0, 0.0, 0.0]),
    )
    sample = ImuSample(t=1.0, accel=[0.1, -0.2, 9.8], gyro=[1.0, 2.0, 3.0])

    result = apply_calibration(sample, coeffs)

    assert result.t == sample.t
    assert np.allclose(result.accel, sample.accel)
    assert np.allclose(result.gyro, sample.gyro)


def test_apply_calibration_applies_bias_and_scale():
    coeffs = CalibrationCoefficients(
        accel_misalignment=np.eye(3),
        accel_scale=np.array([2.0, 3.0, 4.0]),
        accel_bias=np.array([1.0, 1.0, 1.0]),
        gyro_misalignment=np.eye(3),
        gyro_scale=np.array([1.0, 1.0, 1.0]),
        gyro_bias=np.array([0.0, 0.0, 0.0]),
    )
    sample = ImuSample(t=0.0, accel=[0.0, 0.0, 0.0], gyro=[0.0, 0.0, 0.0])

    result = apply_calibration(sample, coeffs)

    # (0 + bias) * scale = [1, 1, 1] * [2, 3, 4]
    assert np.allclose(result.accel, [2.0, 3.0, 4.0])


def test_estimate_gyro_bias_negates_stationary_mean():
    samples = np.array([[1.0, 2.0, -1.0], [3.0, 0.0, 1.0]])

    bias = estimate_gyro_bias(samples)

    assert np.allclose(bias, [-2.0, -1.0, 0.0])


def _reference_calibrate(accel, gyro, gyro_bias):
    """Independent reference implementation of the misalignment/scale/bias
    transform, kept only to regression-test against real coefficients."""
    c_a = _A_COEFF[6:]
    Ta = np.array([[1, -_A_COEFF[0], _A_COEFF[1]], [0, 1, -_A_COEFF[2]], [0, 0, 1]])
    Ka = np.array([[_A_COEFF[3], 0, 0], [0, _A_COEFF[4], 0], [0, 0, _A_COEFF[5]]])

    c_g = gyro_bias
    Tg = np.array(
        [
            [1, -_G_COEFF[0], _G_COEFF[1]],
            [_G_COEFF[2], 1, -_G_COEFF[3]],
            [-_G_COEFF[4], _G_COEFF[5], 1],
        ]
    )
    Kg = np.array([[_G_COEFF[6], 0, 0], [0, _G_COEFF[7], 0], [0, 0, _G_COEFF[8]]])

    a_cal = Ta @ Ka @ np.add(accel, c_a)
    g_cal = Tg @ Kg @ np.add(gyro, c_g)

    return a_cal, g_cal


def test_apply_calibration_matches_reference_formula():
    gyro_bias = np.array([0.01, -0.02, 0.03])
    coeffs = CalibrationCoefficients.from_raw_coefficients(_A_COEFF, _G_COEFF, gyro_bias)
    sample = ImuSample(t=5.0, accel=[0.05, -0.03, 9.79], gyro=[0.5, -1.2, 0.3])

    result = apply_calibration(sample, coeffs)
    expected_accel, expected_gyro = _reference_calibrate(sample.accel, sample.gyro, gyro_bias)

    assert np.allclose(result.accel, expected_accel)
    assert np.allclose(result.gyro, expected_gyro)
