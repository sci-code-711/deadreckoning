# Deadreckoning
A Python toolkit to reconstruct trajectories from Inertial Measurement Unit
readings (gyroscope and accelerometer), by direct integration or Extended
Kalman Filtering (EKF) with attitude correction from gravity vector
measurements.

## Installation

Dependencies are managed with [uv](https://docs.astral.sh/uv/) and declared
in `pyproject.toml`.

```bash
uv sync              # install dependencies into .venv
uv run ruff check .  # lint
uv run ruff format . # format
uv run pytest        # run tests
```

## Library usage

- `deadrec.quaternion.Quaternion` — attitude representation, see
  [wiki:Quaternion](https://en.wikipedia.org/wiki/Quaternion).
- `deadrec.samples.ImuSample` / `TrajectoryState` — the data types
  everything else in the package takes and returns.
- `deadrec.attitude` — gravity-vector attitude estimation
  (`initial_attitude_from_gravity`, `estimate_gravity_magnitude`).
- `deadrec.calibration` — accelerometer/gyroscope misalignment, scale and
  bias correction (`CalibrationCoefficients`, `apply_calibration`).
- `deadrec.filtering` — low-pass filtering by moving average
  (`moving_average_filter`).
- `deadrec.dead_reckoning.DeadReckoner` — trajectory reconstruction by
  direct integration of gyroscope and accelerometer readings.
- `deadrec.ekf.GravityCorrectedEKF` / `WindowedGravityCorrectedEKF` —
  `DeadReckoner` variants that additionally fuse in a gravity-vector
  attitude correction, either from the current reading alone or averaged
  over a window of surrounding readings.
- `deadrec.io` — reading `ImuSample`s from, and writing reconstructed
  trajectories to, CSV files.

```python
from deadrec.attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from deadrec.ekf import GravityCorrectedEKF
from deadrec.io import read_imu_csv, write_trajectory_csv

samples = read_imu_csv("readings.csv")
accel = [s.accel for s in samples]
initial_attitude = initial_attitude_from_gravity(accel[:30])
gravity_magnitude = estimate_gravity_magnitude(accel[:400])

reckoner = GravityCorrectedEKF(initial_attitude, gravity_magnitude)
states = reckoner.run(samples)
write_trajectory_csv(states, "trajectory.csv")
```

## Command-line usage

```bash
uv run deadrec run --in readings.csv --out trajectory.csv --method simple
uv run deadrec run --in readings.csv --out trajectory.csv --method ekf
uv run deadrec run --in readings.csv --out trajectory.csv --method ekf-fut

# See all options, including optional filtering/calibration and tuning the
# gravity-correction methods:
uv run deadrec run --help
```

## Details

The mathematical basis for the trajectory reconstruction methods used here
can be found in the [accompanying paper](https://github.com/sci-code-711/deadreckoning/blob/main/A035_paper.pdf).

The research for this paper was conducted using a 6 axis LSM6SD3 IMU embedded
on an Arduino 33 IoT board.

## Bibliography

[1] D. Tedaldi, A. Pretto, and E. Menegatti, A robust and easy to implement method for imu calibration without external equipments, 2014.
