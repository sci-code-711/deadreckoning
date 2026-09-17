import csv
import json
from pathlib import Path

import numpy as np
import pytest

from deadrec.attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from deadrec.cli import main
from deadrec.dead_reckoning import DeadReckoner
from deadrec.ekf import GravityCorrectedEKF, WindowedGravityCorrectedEKF
from deadrec.io import read_imu_csv

_EXAMPLE_DATA = Path(__file__).resolve().parent.parent / "example_data" / "Example_data.csv"


def test_run_simple_matches_direct_deadreckoner_use(tmp_path):
    out_path = tmp_path / "out.csv"

    exit_code = main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(out_path),
            "--method",
            "simple",
            "--time-divisor",
            "1000",
        ]
    )

    assert exit_code == 0

    samples = read_imu_csv(_EXAMPLE_DATA)
    samples = [type(s)(t=s.t / 1000, accel=s.accel, gyro=s.gyro) for s in samples]
    accel = np.array([s.accel for s in samples])
    initial_attitude = initial_attitude_from_gravity(accel[:30])
    gravity_magnitude = estimate_gravity_magnitude(accel[:400])
    expected_states = DeadReckoner(initial_attitude, gravity_magnitude).run(samples)

    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert len(rows) == len(expected_states)
    last_row, last_state = rows[-1], expected_states[-1]
    assert float(last_row["x"]) == pytest.approx(last_state.position[0])
    assert float(last_row["y"]) == pytest.approx(last_state.position[1])
    assert float(last_row["z"]) == pytest.approx(last_state.position[2])
    assert float(last_row["qw"]) == pytest.approx(last_state.attitude.w)


@pytest.mark.parametrize(
    "method,cls",
    [("ekf", GravityCorrectedEKF), ("ekf-fut", WindowedGravityCorrectedEKF)],
)
def test_run_ekf_methods_match_direct_use(tmp_path, method, cls):
    out_path = tmp_path / "out.csv"

    exit_code = main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(out_path),
            "--method",
            method,
            "--time-divisor",
            "1000",
        ]
    )

    assert exit_code == 0

    samples = read_imu_csv(_EXAMPLE_DATA)
    samples = [type(s)(t=s.t / 1000, accel=s.accel, gyro=s.gyro) for s in samples]
    accel = np.array([s.accel for s in samples])
    initial_attitude = initial_attitude_from_gravity(accel[:30])
    gravity_magnitude = estimate_gravity_magnitude(accel[:400])
    expected_states = cls(initial_attitude, gravity_magnitude).run(samples)

    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert len(rows) == len(expected_states)
    last_row, last_state = rows[-1], expected_states[-1]
    assert float(last_row["x"]) == pytest.approx(last_state.position[0])
    assert float(last_row["qw"]) == pytest.approx(last_state.attitude.w)


def test_run_without_time_divisor_produces_unscaled_dt(tmp_path):
    # Example_data.csv's timestamps are in milliseconds; without
    # --time-divisor they're used as-is, so dt between the first two rows
    # is ~10 (not ~0.01) and the trajectory blows up enormously by the
    # second sample. This just documents that --time-divisor matters, not
    # that the blow-up itself is desirable.
    out_path = tmp_path / "out.csv"

    main(["run", "--in", str(_EXAMPLE_DATA), "--out", str(out_path), "--method", "simple"])

    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert abs(float(rows[1]["x"])) > 1.0


def test_run_filter_range_changes_output(tmp_path):
    unfiltered_path = tmp_path / "unfiltered.csv"
    filtered_path = tmp_path / "filtered.csv"

    main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(unfiltered_path),
            "--time-divisor",
            "1000",
        ]
    )
    main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(filtered_path),
            "--time-divisor",
            "1000",
            "--filter-range",
            "2",
        ]
    )

    with open(unfiltered_path, newline="") as f:
        unfiltered_rows = list(csv.DictReader(f))
    with open(filtered_path, newline="") as f:
        filtered_rows = list(csv.DictReader(f))

    assert float(unfiltered_rows[-1]["x"]) != pytest.approx(float(filtered_rows[-1]["x"]))


def test_run_identity_calibration_matches_uncalibrated_output(tmp_path):
    uncalibrated_path = tmp_path / "uncalibrated.csv"
    calibrated_path = tmp_path / "calibrated.csv"
    calibration_path = tmp_path / "calibration.json"

    calibration_path.write_text(
        json.dumps(
            {
                "accel_coeffs": [0, 0, 0, 1, 1, 1, 0, 0, 0],
                "gyro_coeffs": [0, 0, 0, 0, 0, 0, 1, 1, 1],
                "gyro_bias": [0, 0, 0],
            }
        )
    )

    main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(uncalibrated_path),
            "--time-divisor",
            "1000",
        ]
    )
    main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(calibrated_path),
            "--time-divisor",
            "1000",
            "--calibration",
            str(calibration_path),
        ]
    )

    assert uncalibrated_path.read_text() == calibrated_path.read_text()


def test_run_nontrivial_calibration_changes_output(tmp_path):
    out_path = tmp_path / "uncalibrated.csv"
    calibrated_path = tmp_path / "calibrated.csv"
    calibration_path = tmp_path / "calibration.json"

    calibration_path.write_text(
        json.dumps(
            {
                "accel_coeffs": [0, 0, 0, 1.1, 1.1, 1.1, 0.01, 0.01, 0.01],
                "gyro_coeffs": [0, 0, 0, 0, 0, 0, 1, 1, 1],
                "gyro_bias": [0, 0, 0],
            }
        )
    )

    main(["run", "--in", str(_EXAMPLE_DATA), "--out", str(out_path), "--time-divisor", "1000"])
    main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(calibrated_path),
            "--time-divisor",
            "1000",
            "--calibration",
            str(calibration_path),
        ]
    )

    assert out_path.read_text() != calibrated_path.read_text()


def test_run_missing_required_args_raises():
    with pytest.raises(SystemExit):
        main(["run", "--in", "in.csv"])


def test_run_interpolator_zoh_runs_cleanly_end_to_end(tmp_path):
    out_path = tmp_path / "out.csv"

    exit_code = main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(out_path),
            "--time-divisor",
            "1000",
            "--interpolator",
            "zoh",
        ]
    )

    assert exit_code == 0
    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert len(rows) > 0


def test_run_simple_with_cubic_hermite_interpolator_fails_cleanly(tmp_path, capsys):
    out_path = tmp_path / "out.csv"

    exit_code = main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(out_path),
            "--method",
            "simple",
            "--time-divisor",
            "1000",
            "--interpolator",
            "cubic-hermite",
        ]
    )

    assert exit_code == 1
    captured = capsys.readouterr()
    assert captured.err.startswith("error:")
    assert not out_path.exists()


def test_run_ekf_fut_with_cubic_hermite_interpolator_succeeds(tmp_path):
    out_path = tmp_path / "out.csv"

    exit_code = main(
        [
            "run",
            "--in",
            str(_EXAMPLE_DATA),
            "--out",
            str(out_path),
            "--method",
            "ekf-fut",
            "--time-divisor",
            "1000",
            "--interpolator",
            "cubic-hermite",
        ]
    )

    assert exit_code == 0
    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert len(rows) > 0
