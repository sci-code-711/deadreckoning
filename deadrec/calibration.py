"""IMU calibration: misalignment/scale/bias correction for raw accelerometer
and gyroscope readings.

Ported from the root-level ``Calibration.py``. See ``CalibrationCoefficients``
for the one deliberate behavioural change from the original script.

"""

from dataclasses import dataclass

import numpy as np

from .samples import ImuSample


@dataclass
class CalibrationCoefficients:
    """
    Misalignment, scale, and bias correction terms for one sensor triad.
    Corrected reading is ``misalignment @ (scale * (raw + bias))``.

    Args:
        * accel_misalignment {``np.ndarray``} -- (3, 3) accelerometer
          misalignment correction matrix.
        * accel_scale {``np.ndarray``} -- (3,) accelerometer scale factors.
        * accel_bias {``np.ndarray``} -- (3,) accelerometer bias offset.
        * gyro_misalignment {``np.ndarray``} -- (3, 3) gyroscope misalignment
          correction matrix.
        * gyro_scale {``np.ndarray``} -- (3,) gyroscope scale factors.
        * gyro_bias {``np.ndarray``} -- (3,) gyroscope bias offset, typically
          from :func:`estimate_gyro_bias`.

    """

    accel_misalignment: np.ndarray
    accel_scale: np.ndarray
    accel_bias: np.ndarray
    gyro_misalignment: np.ndarray
    gyro_scale: np.ndarray
    gyro_bias: np.ndarray

    @classmethod
    def from_raw_coefficients(cls, accel_coeffs, gyro_coeffs, gyro_bias):
        """
        Build calibration coefficients from the flat 9-element coefficient
        arrays used in ``Calibration.py`` (``a_coeff``/``g_coeff``).

        Args:
            * accel_coeffs {``array-like``} -- 9 accelerometer coefficients:
              3 misalignment terms, 3 scale factors, 3 bias terms.
            * gyro_coeffs {``array-like``} -- 9 gyroscope coefficients: 3
              misalignment terms (matching a non-triangular layout, unlike
              the accelerometer's), 3 more misalignment terms, then 3 scale
              factors.
            * gyro_bias {``array-like``} -- (3,) gyroscope bias, from
              :func:`estimate_gyro_bias`. Unlike the other coefficients this
              is not a fixed sensor property - it's measured per-session
              from a stationary period, so it isn't packed into
              ``gyro_coeffs``.

        Returns:
            * {``CalibrationCoefficients``}

        """
        accel_coeffs = np.asarray(accel_coeffs, dtype=float)
        gyro_coeffs = np.asarray(gyro_coeffs, dtype=float)
        if accel_coeffs.shape != (9,):
            raise ValueError(f"accel_coeffs must have shape (9,), got {accel_coeffs.shape}")
        if gyro_coeffs.shape != (9,):
            raise ValueError(f"gyro_coeffs must have shape (9,), got {gyro_coeffs.shape}")

        accel_misalignment = np.array(
            [
                [1, -accel_coeffs[0], accel_coeffs[1]],
                [0, 1, -accel_coeffs[2]],
                [0, 0, 1],
            ]
        )
        gyro_misalignment = np.array(
            [
                [1, -gyro_coeffs[0], gyro_coeffs[1]],
                [gyro_coeffs[2], 1, -gyro_coeffs[3]],
                [-gyro_coeffs[4], gyro_coeffs[5], 1],
            ]
        )

        return cls(
            accel_misalignment=accel_misalignment,
            accel_scale=accel_coeffs[3:6],
            accel_bias=accel_coeffs[6:9],
            gyro_misalignment=gyro_misalignment,
            gyro_scale=gyro_coeffs[6:9],
            gyro_bias=np.asarray(gyro_bias, dtype=float),
        )


def estimate_gyro_bias(stationary_gyro_samples) -> np.ndarray:
    """
    Estimate gyroscope zero-rate bias from a period where the sensor was
    known to be stationary, by averaging its readings and negating the
    result (so that ``reading + bias`` cancels out).

    Args:
        * stationary_gyro_samples {``array-like``} -- An (N, 3) array of
          gyroscope readings taken while stationary.

    Returns:
        * {``np.ndarray``} -- (3,) gyroscope bias.

    """
    samples = np.asarray(stationary_gyro_samples, dtype=float)
    return -np.mean(samples, axis=0)


def apply_calibration(sample: ImuSample, coeffs: CalibrationCoefficients) -> ImuSample:
    """
    Apply misalignment/scale/bias calibration to one IMU reading.

    Args:
        * sample {``ImuSample``} -- The raw reading to calibrate. ``t`` is
          passed through unchanged - timestamp normalisation is a separate
          concern from sensor calibration.
        * coeffs {``CalibrationCoefficients``} -- The calibration to apply.

    Returns:
        * {``ImuSample``} -- A new, calibrated sample.

    """
    accel = coeffs.accel_misalignment @ (coeffs.accel_scale * (sample.accel + coeffs.accel_bias))
    gyro = coeffs.gyro_misalignment @ (coeffs.gyro_scale * (sample.gyro + coeffs.gyro_bias))

    return ImuSample(t=sample.t, accel=accel, gyro=gyro)
