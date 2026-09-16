import time
from multiprocessing import Queue

import pytest

from deadrec.calibration import CalibrationCoefficients, apply_calibration
from deadrec.dead_reckoning import DeadReckoner
from deadrec.ekf import GravityCorrectedEKF
from deadrec.io import trajectory_state_to_row
from deadrec.quaternion import Quaternion
from deadrec.samples import ImuSample
from deadrec.transformers import ReconstructionTransformer


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.exceptions = []

    def info(self, *args, **kwargs):
        self.infos.append(args)

    def warning(self, *args, **kwargs):
        self.warnings.append(args)

    def exception(self, *args, **kwargs):
        self.exceptions.append(args)


def _make_transformer(reckoner):
    return ReconstructionTransformer(Queue(), Queue(), reckoner=reckoner)


def test_transformation_matches_direct_step_for_dead_reckoner():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    reference = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)

    samples = [
        ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]),
        ImuSample(t=0.1, accel=[0, 0, 1], gyro=[1, 0, 0]),
        ImuSample(t=0.2, accel=[0.1, 0, 1], gyro=[0, 2, 0]),
    ]

    for sample in samples:
        row = transformer.transformation(sample)
        expected_row = [str(value) for value in trajectory_state_to_row(reference.step(sample))]
        assert row == expected_row


def test_transformation_works_with_ekf_reckoner():
    reckoner = GravityCorrectedEKF(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    reference = GravityCorrectedEKF(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)

    sample = ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0])

    row = transformer.transformation(sample)
    expected_row = [str(value) for value in trajectory_state_to_row(reference.step(sample))]
    assert row == expected_row


def test_transformation_applies_calibration_before_stepping():
    coeffs = CalibrationCoefficients.from_raw_coefficients(
        accel_coeffs=[0, 0, 0, 1.1, 1.1, 1.1, 0.01, 0.01, 0.01],
        gyro_coeffs=[0, 0, 0, 0, 0, 0, 1, 1, 1],
        gyro_bias=[0, 0, 0],
    )
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    reference = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = ReconstructionTransformer(Queue(), Queue(), reckoner=reckoner, calibration=coeffs)

    sample = ImuSample(t=0.0, accel=[0, 0, 1], gyro=[1, 2, 3])

    row = transformer.transformation(sample)
    expected_row = [
        str(value)
        for value in trajectory_state_to_row(reference.step(apply_calibration(sample, coeffs)))
    ]

    assert row == expected_row
    # And it must actually differ from the uncalibrated result - otherwise
    # this test wouldn't catch calibration silently not being applied.
    uncalibrated_row = [
        str(value)
        for value in trajectory_state_to_row(
            DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0).step(sample)
        )
    ]
    assert row != uncalibrated_row


def test_transformation_without_calibration_uses_raw_sample():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    reference = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)

    sample = ImuSample(t=0.0, accel=[0, 0, 1], gyro=[1, 2, 3])

    row = transformer.transformation(sample)
    expected_row = [str(value) for value in trajectory_state_to_row(reference.step(sample))]

    assert row == expected_row


def test_transformation_without_a_logger_does_not_raise():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)

    transformer.transformation(ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]))
    transformer.transformation(ImuSample(t=0.1, accel=[0, 0, 1], gyro=[0, 0, 0]))


def test_transformation_warns_when_exceeding_latency_threshold(monkeypatch):
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)
    transformer.logger = _FakeLogger()

    # First sample only seeds _prev_t - there's no interval to compare against yet.
    transformer.transformation(ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]))
    assert transformer.logger.warnings == []

    original_step = reckoner.step

    def slow_step(sample):
        time.sleep(0.02)
        return original_step(sample)

    monkeypatch.setattr(reckoner, "step", slow_step)

    # dt to this sample is 1ms, but the (patched) step takes ~20ms - well past
    # the default 0.5 threshold, without even needing to exceed the full dt.
    transformer.transformation(ImuSample(t=0.001, accel=[0, 0, 1], gyro=[0, 0, 0]))

    assert len(transformer.logger.warnings) == 1


def test_transformation_does_not_warn_when_keeping_up():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)
    transformer.logger = _FakeLogger()

    transformer.transformation(ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]))
    transformer.transformation(ImuSample(t=1.0, accel=[0, 0, 1], gyro=[0, 0, 0]))

    assert transformer.logger.warnings == []


def test_transformation_never_logs_per_step_info():
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = _make_transformer(reckoner)
    transformer.logger = _FakeLogger()

    transformer.transformation(ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]))
    transformer.transformation(ImuSample(t=0.1, accel=[0, 0, 1], gyro=[0, 0, 0]))

    assert transformer.logger.infos == []


def test_transformation_latency_warning_threshold_is_configurable(monkeypatch):
    reckoner = DeadReckoner(Quaternion(1, 0, 0, 0), gravity_magnitude=1.0)
    transformer = ReconstructionTransformer(
        Queue(), Queue(), reckoner=reckoner, latency_warning_threshold=0.9
    )
    transformer.logger = _FakeLogger()

    transformer.transformation(ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]))

    original_step = reckoner.step

    def slow_step(sample):
        time.sleep(0.02)
        return original_step(sample)

    monkeypatch.setattr(reckoner, "step", slow_step)

    # dt is 100ms; the ~20ms (patched) step exceeds the default 0.5 threshold
    # (50ms) but not this transformer's 0.9 threshold (90ms).
    transformer.transformation(ImuSample(t=0.1, accel=[0, 0, 1], gyro=[0, 0, 0]))

    assert transformer.logger.warnings == []


def test_transformation_logs_and_reraises_on_exception():
    class FailingReckoner:
        def step(self, sample):
            raise ValueError("boom")

    transformer = _make_transformer(FailingReckoner())
    transformer.logger = _FakeLogger()

    with pytest.raises(ValueError, match="boom"):
        transformer.transformation(ImuSample(t=0.0, accel=[0, 0, 1], gyro=[0, 0, 0]))

    assert len(transformer.logger.exceptions) == 1
