"""Reading IMU samples from, and writing reconstructed trajectories to, CSV
files.

"""

import csv

from .samples import ImuSample, TrajectoryState

_TRAJECTORY_COLUMNS = [
    "t",
    "qw",
    "qx",
    "qy",
    "qz",
    "ax",
    "ay",
    "az",
    "vx",
    "vy",
    "vz",
    "x",
    "y",
    "z",
    "phi",
    "m",
    "n",
]


def imu_sample_from_row(row: dict) -> ImuSample:
    """
    Build a single :class:`ImuSample` from one CSV row, keyed as produced by
    :class:`csv.DictReader` on a file with columns ``t, ax, ay, az, vl, vm,
    vn`` (timestamp, accelerometer x/y/z, gyroscope x/y/z) - the format used
    by ``example_data/Example_data.csv``.

    Args:
        * row {``dict``} -- A single CSV row, keyed by column name.

    Returns:
        * {``ImuSample``} -- The parsed sample.

    """
    return ImuSample(
        t=float(row["t"]),
        accel=[float(row["ax"]), float(row["ay"]), float(row["az"])],
        gyro=[float(row["vl"]), float(row["vm"]), float(row["vn"])],
    )


def trajectory_state_to_row(state: TrajectoryState) -> list:
    """
    Format a single :class:`TrajectoryState` as one row matching
    ``_TRAJECTORY_COLUMNS``'s column order: timestamp, attitude quaternion,
    gravity-removed navigation-frame acceleration, velocity, position, and
    attitude as Euler angles.

    Args:
        * state {``TrajectoryState``} -- The state to format.

    Returns:
        * {``list``} -- The row's values, in ``_TRAJECTORY_COLUMNS`` order,
          not yet converted to strings.

    """
    return [
        state.t,
        state.attitude.w,
        state.attitude.x,
        state.attitude.y,
        state.attitude.z,
        *state.accel_nav,
        *state.velocity,
        *state.position,
        *state.euler,
    ]


def read_imu_csv(path) -> list[ImuSample]:
    """
    Read IMU samples from a CSV file with columns ``t, ax, ay, az, vl, vm,
    vn`` (timestamp, accelerometer x/y/z, gyroscope x/y/z) - the format used
    by ``example_data/Example_data.csv``.

    Args:
        * path {``str`` or ``Path``} -- Path to the CSV file to read.

    Returns:
        * {``list[ImuSample]``} -- The samples, in file order.

    """
    with open(path, newline="") as csv_file:
        return [imu_sample_from_row(row) for row in csv.DictReader(csv_file)]


def write_trajectory_csv(states: list[TrajectoryState], path) -> None:
    """
    Write reconstructed trajectory states to a CSV file: timestamp,
    attitude quaternion, gravity-removed navigation-frame acceleration,
    velocity, position, and attitude as Euler angles.

    Args:
        * states {``Iterable[TrajectoryState]``} -- The states to write,
          e.g. from :meth:`deadrec.dead_reckoning.DeadReckoner.run`.
        * path {``str`` or ``Path``} -- Path to write the CSV file to.

    """
    with open(path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(_TRAJECTORY_COLUMNS)

        for state in states:
            writer.writerow(trajectory_state_to_row(state))
