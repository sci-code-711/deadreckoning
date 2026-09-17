"""Command-line interface: run the calibrate/filter/reconstruct pipeline on
a CSV of IMU readings end-to-end.

"""

import argparse
import json
import sys
from multiprocessing import Queue

import numpy as np

from .attitude import estimate_gravity_magnitude, initial_attitude_from_gravity
from .calibration import CalibrationCoefficients, apply_calibration
from .connectors import LiveCSVReplay, ToCSV
from .core import Core
from .dead_reckoning import DeadReckoner
from .ekf import GravityCorrectedEKF, WindowedGravityCorrectedEKF
from .filtering import moving_average_filter
from .io import _TRAJECTORY_COLUMNS, read_imu_csv, write_trajectory_csv
from .logger import QueueLogListener
from .samples import ImuSample
from .transformers import ReconstructionTransformer

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

    stream_parser = subparsers.add_parser(
        "stream",
        help="Replay a CSV as a paced live IMU stream through the real-time "
        "reconstruction pipeline, to prove it keeps up with real-time sample "
        "arrival rather than just being fast in isolation.",
    )
    stream_parser.add_argument(
        "--in", dest="in_path", required=True, help="Input CSV (t, ax, ay, az, vl, vm, vn)."
    )
    stream_parser.add_argument(
        "--out", dest="out_path", required=True, help="Path to write the trajectory CSV to."
    )
    stream_parser.add_argument(
        "--method",
        choices=["simple", "ekf"],
        default="simple",
        help="Reconstruction method: 'simple' (direct integration) or 'ekf' "
        "(instantaneous gravity correction). 'ekf-fut' isn't available here "
        "- it needs a look-ahead window of future samples, so it can't "
        "process one live/streamed sample at a time. Defaults to 'simple'.",
    )
    stream_parser.add_argument(
        "--time-divisor",
        type=float,
        default=1.0,
        help="Divide every sample's timestamp by this before reconstruction "
        "and pacing - e.g. 1000 if the input file's timestamps are in "
        "milliseconds (as in example_data/Example_data.csv). Defaults to 1 "
        "(no conversion).",
    )
    stream_parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Replay speed multiplier: 1.0 (the default) paces samples at "
        "real wall-clock time; higher values replay faster, e.g. to finish "
        "a demo or test run early.",
    )
    stream_parser.add_argument(
        "--calibration",
        default=None,
        help="Path to a JSON file with accel_coeffs (9 numbers), gyro_coeffs "
        "(9 numbers) and gyro_bias (3 numbers) - see "
        "CalibrationCoefficients.from_raw_coefficients. If omitted, "
        "readings are used uncalibrated. Note: --filter-range isn't "
        "available for streaming, since low-pass filtering needs the whole "
        "sample array up front rather than one sample at a time.",
    )
    stream_parser.add_argument(
        "--attitude-samples",
        type=int,
        default=30,
        help="Number of leading samples used to estimate initial attitude. Defaults to 30.",
    )
    stream_parser.add_argument(
        "--gravity-samples",
        type=int,
        default=400,
        help="Number of leading samples used to estimate gravity magnitude. Defaults to 400.",
    )
    stream_parser.add_argument(
        "--deviation-threshold",
        type=float,
        default=None,
        help="Gravity-deviation gate for --method ekf. Defaults to the method's own default.",
    )
    stream_parser.add_argument(
        "--beta",
        type=float,
        default=None,
        help="Gravity-correction blend weight for --method ekf. Defaults to "
        "the method's own default.",
    )
    stream_parser.add_argument(
        "--latency-warning-threshold",
        type=float,
        default=0.5,
        help="Warn when a step's processing time exceeds this proportion of "
        "the real time gap since the previous sample - an early signal that "
        "reconstruction may be falling behind real-time arrival. Defaults to 0.5.",
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


def _estimate_initial_state(samples: list[ImuSample], attitude_samples: int, gravity_samples: int):
    """
    Estimate the initial attitude and gravity magnitude a reckoner should be
    seeded with, from the leading accelerometer readings of ``samples``.
    Depends only on accelerometer values, not on timing, so it's the same
    whether ``samples`` have already been time-divided or not.

    Returns:
        * {``tuple``} -- ``(initial_attitude, gravity_magnitude)``.

    """
    accel = np.array([sample.accel for sample in samples])
    initial_attitude = initial_attitude_from_gravity(accel[:attitude_samples])
    gravity_magnitude = estimate_gravity_magnitude(accel[:gravity_samples])

    return initial_attitude, gravity_magnitude


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

    initial_attitude, gravity_magnitude = _estimate_initial_state(
        samples, args.attitude_samples, args.gravity_samples
    )

    reckoner = _build_reckoner(args, initial_attitude, gravity_magnitude)
    states = reckoner.run(samples)

    write_trajectory_csv(states, args.out_path)


def stream(args: argparse.Namespace) -> None:
    """
    Run the reconstruction pipeline for the ``stream`` subcommand: replay
    ``--in`` as a paced, live-simulated IMU stream through the real
    multiprocessing pipeline (:class:`deadrec.core.Core`), rather than
    processing it as one in-memory batch like :func:`run` does.

    """
    # This upfront read is only used to seed the reckoner's initial attitude
    # and gravity magnitude, which depend only on the leading accelerometer
    # readings, not on timing - LiveCSVReplay independently re-parses the
    # same file below for the actual paced, streamed run.
    seed_samples = read_imu_csv(args.in_path)
    initial_attitude, gravity_magnitude = _estimate_initial_state(
        seed_samples, args.attitude_samples, args.gravity_samples
    )
    reckoner = _build_reckoner(args, initial_attitude, gravity_magnitude)

    calibration = _load_calibration(args.calibration) if args.calibration is not None else None

    log_queue = Queue()
    listener = QueueLogListener(log_queue)
    listener.start()
    try:
        core = Core("stream")
        core.set_input_connector(
            LiveCSVReplay(args.in_path, time_divisor=args.time_divisor, speed=args.speed)
        )
        core.set_output_connector(ToCSV(args.out_path, header=_TRAJECTORY_COLUMNS))
        core.transformer = ReconstructionTransformer
        core.transformer_kwargs = {
            "reckoner": reckoner,
            "log_queue": log_queue,
            "latency_warning_threshold": args.latency_warning_threshold,
            "calibration": calibration,
        }
        core.launch()
        core.terminate()
    finally:
        listener.stop()


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    if args.command == "run":
        run(args)
    elif args.command == "stream":
        stream(args)

    return 0


if __name__ == "__main__":
    sys.exit(main())
