import numpy as np
import pytest

from deadrec.filtering import moving_average_filter


def _brute_force_filter(data: np.ndarray, filt_range: int) -> np.ndarray:
    data = np.asarray(data, dtype=float)
    n = data.shape[0]
    result = np.empty_like(data)

    for r in range(n):
        lo = max(0, r - filt_range)
        hi = min(n, r + filt_range + 1)
        result[r] = data[lo:hi].mean(axis=0)

    return result


def test_moving_average_filter_known_values():
    data = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    result = moving_average_filter(data, filt_range=1)

    assert np.allclose(result, [1.5, 2.0, 3.0, 4.0, 5.0, 5.5])


def test_moving_average_filter_matches_brute_force_symmetric_window():
    rng = np.random.default_rng(0)
    data = rng.normal(size=(25, 3))

    for filt_range in (0, 1, 3, 5):
        result = moving_average_filter(data, filt_range)
        expected = _brute_force_filter(data, filt_range)

        assert np.allclose(result, expected), f"mismatch for filt_range={filt_range}"


def test_moving_average_filter_zero_range_is_identity():
    data = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])

    result = moving_average_filter(data, filt_range=0)

    assert np.allclose(result, data)


def test_moving_average_filter_single_column():
    data = np.array([2.0, 4.0, 6.0, 8.0])

    result = moving_average_filter(data, filt_range=1)

    assert np.allclose(result, _brute_force_filter(data, 1))


def test_moving_average_filter_range_at_least_data_length_raises():
    # Mirrors an existing limitation of Filter.py: the first-row window
    # reads filt_range + 1 rows unconditionally, so filt_range must be
    # smaller than the data length.
    data = np.array([1.0, 2.0, 3.0])

    with pytest.raises(IndexError):
        moving_average_filter(data, filt_range=5)
