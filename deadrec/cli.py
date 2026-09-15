"""Command-line interface: run the calibrate/filter/reconstruct pipeline on
a CSV of IMU readings end-to-end.

"""

import argparse
import json
import sys

import numpy as np

from .attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from .calibration import CalibrationCoefficients, apply_calibration
from .dead_reckoning import DeadReckoner
from .ekf import GravityCorrectedEKF, WindowedGravityCorrectedEKF
from .filtering import moving_average_filter
from .io import read_imu_csv, write_trajectory_csv
from .samples import ImuSample

_METHODS = {
    "simple": DeadReckoner,
    "ekf": GravityCorrectedEKF,
    "ekf-fut": WindowedGravityCorrectedEKF,
}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="deadrec", description="Reconstruct a trajectory from a CSV of IMU readings."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="Run the reconstruction pipeline.")
    run_parser.add_argument(
        "--in", dest="in_path", required=True, help="Input CSV (t, ax, ay, az, vl, vm, vn)."
    )
    run_parser.add_argument(
        "--out", dest="out_path", required=True, help="Path to write the trajectory CSV to."
    )
    run_parser.add_argument(
        "--method",
        choices=sorted(_METHODS),
        default="simple",
        help="Reconstruction method: 'simple' (direct integration), 'ekf' "
        "(instantaneous gravity correction), or 'ekf-fut' (windowed gravity "
        "correction). Defaults to 'simple'.",
    )
    run_parser.add_argument(
        "--time-divisor",
        type=float,
        default=1.0,
        help="Divide every sample's timestamp by this before reconstruction "
        "- e.g. 1000 if the input file's timestamps are in milliseconds "
        "but accelerometer/gyroscope units imply seconds (as in "
        "example_data/Example_data.csv). Defaults to 1 (no conversion).",
    )
    run_parser.add_argument(
        "--filter-range",
        type=int,
        default=None,
        help="If set, low-pass filter accelerometer/gyroscope readings with "
        "this window radius before reconstruction.",
    )
    run_parser.add_argument(
        "--calibration",
        default=None,
        help="Path to a JSON file with accel_coeffs (9 numbers), gyro_coeffs "
        "(9 numbers) and gyro_bias (3 numbers) - see "
        "CalibrationCoefficients.from_raw_coefficients. If omitted, "
        "readings are used uncalibrated.",
    )
    run_parser.add_argument(
        "--attitude-samples",
        type=int,
        default=30,
        help="Number of leading samples used to estimate initial attitude. Defaults to 30.",
    )
    run_parser.add_argument(
        "--gravity-samples",
        type=int,
        default=400,
        help="Number of leading samples used to estimate gravity magnitude. Defaults to 400.",
    )
    run_parser.add_argument(
        "--deviation-threshold",
        type=float,
        default=None,
        help="Gravity-deviation gate for --method ekf/ekf-fut. Defaults to "
        "each method's own default.",
    )
    run_parser.add_argument(
        "--beta",
        type=float,
        default=None,
        help="Gravity-correction blend weight for --method ekf/ekf-fut. "
        "Defaults to each method's own default.",
    )
    run_parser.add_argument(
        "--window-radius",
        type=int,
        default=None,
        help="Look-ahead/behind window radius for --method ekf-fut. "
        "Defaults to that method's own default.",
    )

    return parser


def _load_calibration(path: str) -> CalibrationCoefficients:
    with open(path) as calibration_file:
        data = json.load(calibration_file)

    return CalibrationCoefficients.from_raw_coefficients(
        data["accel_coeffs"], data["gyro_coeffs"], data["gyro_bias"]
    )


def _filter_samples(samples: list[ImuSample], filter_range: int) -> list[ImuSample]:
    accel = moving_average_filter(np.array([s.accel for s in samples]), filter_range)
    gyro = moving_average_filter(np.array([s.gyro for s in samples]), filter_range)

    return [ImuSample(t=s.t, accel=accel[i], gyro=gyro[i]) for i, s in enumerate(samples)]


def _build_reckoner(args: argparse.Namespace, initial_attitude, gravity_magnitude):
    cls = _METHODS[args.method]
    kwargs = {}

    if args.method != "simple":
        if args.deviation_threshold is not None:
            kwargs["deviation_threshold"] = args.deviation_threshold
        if args.beta is not None:
            kwargs["beta"] = args.beta
    if args.method == "ekf-fut" and args.window_radius is not None:
        kwargs["window_radius"] = args.window_radius

    return cls(initial_attitude, gravity_magnitude, **kwargs)


def run(args: argparse.Namespace) -> None:
    """Run the calibrate/filter/reconstruct pipeline for the ``run`` subcommand."""
    samples = read_imu_csv(args.in_path)

    if args.time_divisor != 1.0:
        samples = [
            ImuSample(t=sample.t / args.time_divisor, accel=sample.accel, gyro=sample.gyro)
            for sample in samples
        ]

    if args.filter_range is not None:
        samples = _filter_samples(samples, args.filter_range)

    if args.calibration is not None:
        coeffs = _load_calibration(args.calibration)
        samples = [apply_calibration(sample, coeffs) for sample in samples]

    accel = np.array([sample.accel for sample in samples])
    initial_attitude = initial_attitude_from_gravity(accel[: args.attitude_samples])
    gravity_magnitude = estimate_gravity_magnitude(accel[: args.gravity_samples])

    reckoner = _build_reckoner(args, initial_attitude, gravity_magnitude)
    states = reckoner.run(samples)

    write_trajectory_csv(states, args.out_path)


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    if args.command == "run":
        run(args)

    return 0


if __name__ == "__main__":
    sys.exit(main())
