"""Low-pass filtering of IMU readings by moving average.

Ported from the root-level ``Filter.py``.

"""

import numpy as np


def moving_average_filter(data, filt_range: int) -> np.ndarray:
    """
    Low-pass filter ``data`` by taking, for each row, the average of a
    symmetric window of ``2 * filt_range + 1`` rows centred on it. Near
    either edge of the array the window is truncated to whatever rows
    exist (not zero-padded), so it shrinks rather than including
    out-of-bounds data.

    ``data`` should contain only the columns to be filtered - the original
    ``Filter.py`` always excluded its time column before filtering, and
    callers here should do the same (e.g. filter accelerometer/gyroscope
    columns only, and leave timestamps untouched).

    Args:
        * data {``array-like``} -- An (N, ...) array of readings to filter,
          one row per sample.
        * filt_range {``int``} -- Number of rows either side of each row to
          average over.

    Returns:
        * {``np.ndarray``} -- The filtered array, the same shape as ``data``.

    """
    data = np.asarray(data, dtype=float)
    n = data.shape[0]
    result = np.empty_like(data)
    total = np.zeros(data.shape[1:], dtype=float)

    for r in range(n):
        if r == 0:
            for a in range(filt_range + 1):
                total = total + data[a]
            result[0] = total / (filt_range + 1)
        elif r < filt_range + 1:
            total = total + data[r + filt_range]
            result[r] = total / (r + filt_range + 1)
        elif r > n - filt_range - 1:
            total = total - data[r - filt_range - 1]
            result[r] = total / (n - r + filt_range)
        else:
            total = total + data[r + filt_range]
            total = total - data[r - filt_range - 1]
            result[r] = total / (2 * filt_range + 1)

    return result
