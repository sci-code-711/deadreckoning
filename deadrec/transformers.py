from .runners import Runner, TerminateSignal
from multiprocessing import Queue
from abc import ABC, abstractmethod
from time import monotonic

from .calibration import CalibrationCoefficients, apply_calibration
from .io import trajectory_state_to_row
from .logger import get_worker_logger
from .samples import ImuSample


class TransformerBase(Runner, ABC):
    def __init__(self, i_stream: Queue, o_stream: Queue):
        self.i_stream = i_stream
        self.o_stream = o_stream
        super().__init__()

    def run(self):
        count = 0
        while True:
            item = self.i_stream.get()
            if isinstance(item, TerminateSignal):
                self.o_stream.put(item)
                break

            count += 1
            try:
                result = self.transformation(item)
            except Exception as exc:
                # Forward the failure downstream before dying, so an output
                # connector blocked on i_stream.get() doesn't wait forever
                # for a TerminateSignal that would otherwise never come.
                self.o_stream.put(TerminateSignal(False, exc))
                raise

            self.o_stream.put(result)

        print(f"Transformer has finished processing {count} items")

        return True

    @abstractmethod
    def transformation(item):
        pass


class ReconstructionTransformer(TransformerBase):
    """
    A :class:`TransformerBase` that wraps a streaming-capable reckoner (a
    :class:`deadrec.dead_reckoning.DeadReckoner` or
    :class:`deadrec.ekf.GravityCorrectedEKF` instance, which both have a
    working ``.step()``) to reconstruct a trajectory one live/streamed
    :class:`deadrec.samples.ImuSample` at a time.

    Args:
        * i_stream, o_stream -- See :class:`TransformerBase`.
        * reckoner -- The reckoner instance to drive with ``.step()``. Its
          own configuration (initial attitude, gravity magnitude, etc.) is
          the caller's responsibility.
        * log_queue {``multiprocessing.Queue``} -- If given, a worker logger
          (see :func:`deadrec.logger.get_worker_logger`) is set up in this
          transformer's own process to warn when a step's processing time
          exceeds ``latency_warning_threshold`` of the real time gap since
          the previous sample - i.e. when the reconstruction is at risk of
          falling behind real-time arrival. If omitted, no logging is done.
        * latency_warning_threshold {``float``} -- Fraction of the sample
          interval a step's processing time must exceed to be logged as a
          warning, e.g. the default ``0.5`` warns once a step takes more
          than half as long as the gap between samples. Has no effect
          without ``log_queue``.
        * calibration {``CalibrationCoefficients``} -- If given, applied to
          each sample (see :func:`deadrec.calibration.apply_calibration`)
          before it's passed to ``reckoner.step()``. Calibration is a
          per-sample operation, so unlike low-pass filtering it works fine
          on a live/streamed sample at a time. Defaults to no calibration.

    """

    def __init__(
        self,
        i_stream: Queue,
        o_stream: Queue,
        *,
        reckoner,
        log_queue: Queue = None,
        latency_warning_threshold: float = 0.5,
        calibration: CalibrationCoefficients = None,
    ):
        self.reckoner = reckoner
        self.log_queue = log_queue
        self.latency_warning_threshold = latency_warning_threshold
        self.calibration = calibration
        self.logger = None
        self._prev_t = None
        super().__init__(i_stream, o_stream)

    def run(self):
        # Logging must be set up here, not in __init__, since __init__ runs
        # in the parent process before this transformer's own process
        # exists - a logging.Logger isn't reliably transferable across that
        # boundary (e.g. under the "spawn" multiprocessing start method).
        if self.log_queue is not None:
            self.logger = get_worker_logger(self.log_queue, "deadrec.stream")

        return super().run()

    def transformation(self, item: ImuSample) -> list:
        if self.calibration is not None:
            item = apply_calibration(item, self.calibration)

        start = monotonic()
        try:
            state = self.reckoner.step(item)
        except Exception:
            if self.logger is not None:
                self.logger.exception("Reconstruction step failed at t=%s", item.t)
            raise
        elapsed = monotonic() - start

        if self.logger is not None and self._prev_t is not None:
            dt = item.t - self._prev_t
            if dt > 0 and elapsed > self.latency_warning_threshold * dt:
                self.logger.warning(
                    "step t=%.6f took %.6fs, exceeding %.0f%% of the %.6fs sample interval - "
                    "reconstruction may be falling behind real-time",
                    item.t,
                    elapsed,
                    self.latency_warning_threshold * 100,
                    dt,
                )
        self._prev_t = item.t

        return [str(value) for value in trajectory_state_to_row(state)]
