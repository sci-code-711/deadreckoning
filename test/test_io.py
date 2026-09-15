import csv
from pathlib import Path

import numpy as np
import pytest

from deadrec.attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from deadrec.dead_reckoning import DeadReckoner
from deadrec.io import read_imu_csv, write_trajectory_csv
from deadrec.quaternion import Quaternion
from deadrec.samples import ImuSample, TrajectoryState


def test_read_imu_csv_reads_example_data():
    path = Path(__file__).resolve().parent.parent / "example_data" / "Example_data.csv"

    samples = read_imu_csv(path)

    assert len(samples) == 2000
    first = samples[0]
    assert first.t == pytest.approx(11991.0)
    assert np.allclose(first.accel, [0.0258, -0.0004, 1.0073])
    assert np.allclose(first.gyro, [3.8452, -4.0894, -2.1362])


def test_write_trajectory_csv_round_trips(tmp_path):
    states = [
        TrajectoryState(
            t=0.0,
            attitude=Quaternion(1, 0, 0, 0),
            accel_nav=[0.1, 0.2, 0.3],
            velocity=[0.0, 0.0, 0.0],
            position=[0.0, 0.0, 0.0],
            euler=(0.0, 0.0, 0.0),
        ),
        TrajectoryState(
            t=0.1,
            attitude=Quaternion.from_eul_angles(0.1, 0.0, 0.0),
            accel_nav=[0.0, 0.1, -0.1],
            velocity=[0.01, 0.02, 0.03],
            position=[0.001, 0.002, 0.003],
            euler=(0.2, 0.0, 0.0),
        ),
    ]
    out_path = tmp_path / "trajectory.csv"

    write_trajectory_csv(states, out_path)

    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert len(rows) == 2
    assert float(rows[0]["t"]) == 0.0
    assert float(rows[0]["qw"]) == 1.0
    assert float(rows[1]["t"]) == pytest.approx(0.1)
    assert float(rows[1]["vx"]) == pytest.approx(0.01)
    assert float(rows[1]["y"]) == pytest.approx(0.002)
    assert float(rows[1]["phi"]) == pytest.approx(0.2)


def test_dead_reckoner_output_round_trips_through_csv(tmp_path):
    path = Path(__file__).resolve().parent.parent / "example_data" / "Example_data.csv"
    samples = read_imu_csv(path)

    initial_attitude = initial_attitude_from_gravity(np.array([s.accel for s in samples[:30]]))
    gravity_magnitude = estimate_gravity_magnitude(np.array([s.accel for s in samples[:400]]))
    states = DeadReckoner(initial_attitude, gravity_magnitude).run(samples)

    out_path = tmp_path / "trajectory.csv"
    write_trajectory_csv(states, out_path)

    with open(out_path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert len(rows) == len(states)
    last_row, last_state = rows[-1], states[-1]
    assert float(last_row["t"]) == pytest.approx(last_state.t)
    assert float(last_row["x"]) == pytest.approx(last_state.position[0])
    assert float(last_row["y"]) == pytest.approx(last_state.position[1])
    assert float(last_row["z"]) == pytest.approx(last_state.position[2])
    assert float(last_row["qw"]) == pytest.approx(last_state.attitude.w)


def test_read_imu_csv_missing_column_raises(tmp_path):
    bad_csv = tmp_path / "bad.csv"
    bad_csv.write_text("t,ax,ay,az,vl,vm\n0,1,2,3,4,5\n")

    with pytest.raises(KeyError):
        read_imu_csv(bad_csv)


def test_write_trajectory_csv_empty_states_writes_header_only(tmp_path):
    out_path = tmp_path / "empty.csv"

    write_trajectory_csv([], out_path)

    with open(out_path, newline="") as csv_file:
        rows = list(csv.reader(csv_file))

    assert len(rows) == 1
    assert rows[0][0] == "t"


def test_read_imu_csv_accepts_imu_sample_of_first_row():
    path = Path(__file__).resolve().parent.parent / "example_data" / "Example_data.csv"

    samples = read_imu_csv(path)

    assert isinstance(samples[0], ImuSample)
