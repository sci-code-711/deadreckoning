import numpy as np

from deadrec.noise import add_gaussian_noise
from deadrec.samples import ImuSample


def _samples(n=5):
    return [ImuSample(t=float(i), accel=[1.0, 2.0, 3.0], gyro=[10.0, 20.0, 30.0]) for i in range(n)]


def test_zero_std_is_a_no_op_passthrough():
    samples = _samples()

    result = add_gaussian_noise(samples, accel_std=0.0, gyro_std=0.0, seed=0)

    for original, noisy in zip(samples, result):
        assert noisy.t == original.t
        assert np.array_equal(noisy.accel, original.accel)
        assert np.array_equal(noisy.gyro, original.gyro)


def test_default_std_is_a_no_op_passthrough():
    samples = _samples()

    result = add_gaussian_noise(samples, seed=0)

    for original, noisy in zip(samples, result):
        assert np.array_equal(noisy.accel, original.accel)
        assert np.array_equal(noisy.gyro, original.gyro)


def test_input_samples_are_not_mutated():
    samples = _samples()
    original_accel = [s.accel.copy() for s in samples]

    add_gaussian_noise(samples, accel_std=1.0, gyro_std=1.0, seed=0)

    for sample, before in zip(samples, original_accel):
        assert np.array_equal(sample.accel, before)


def test_timestamps_are_preserved():
    samples = _samples()

    result = add_gaussian_noise(samples, accel_std=0.5, gyro_std=0.5, seed=0)

    assert [s.t for s in result] == [s.t for s in samples]


def test_noise_statistics_match_requested_std():
    n = 20000
    samples = [ImuSample(t=0.0, accel=[0.0, 0.0, 0.0], gyro=[0.0, 0.0, 0.0]) for _ in range(n)]

    result = add_gaussian_noise(samples, accel_std=0.3, gyro_std=2.0, seed=42)

    accel = np.array([s.accel for s in result])
    gyro = np.array([s.gyro for s in result])

    assert np.allclose(accel.mean(axis=0), 0.0, atol=0.02)
    assert np.allclose(accel.std(axis=0), 0.3, atol=0.02)
    assert np.allclose(gyro.mean(axis=0), 0.0, atol=0.1)
    assert np.allclose(gyro.std(axis=0), 2.0, atol=0.1)


def test_same_seed_is_reproducible():
    samples = _samples()

    result_a = add_gaussian_noise(samples, accel_std=1.0, gyro_std=1.0, seed=7)
    result_b = add_gaussian_noise(samples, accel_std=1.0, gyro_std=1.0, seed=7)

    for a, b in zip(result_a, result_b):
        assert np.array_equal(a.accel, b.accel)
        assert np.array_equal(a.gyro, b.gyro)


def test_different_seeds_produce_different_noise():
    samples = _samples()

    result_a = add_gaussian_noise(samples, accel_std=1.0, gyro_std=1.0, seed=1)
    result_b = add_gaussian_noise(samples, accel_std=1.0, gyro_std=1.0, seed=2)

    assert not all(np.array_equal(a.accel, b.accel) for a, b in zip(result_a, result_b))
