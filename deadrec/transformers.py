from .runners import Runner, TerminateSignal
from multiprocessing import Queue
from abc import ABC, abstractmethod
from time import monotonic

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
            self.o_stream.put(self.transformation(item))

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
          transformer's own process to record per-step processing latency,
          and to warn when a step takes longer than the real time gap since
          the previous sample - i.e. when the reconstruction is falling
          behind real-time arrival. If omitted, no logging is done.

    """

    def __init__(self, i_stream: Queue, o_stream: Queue, *, reckoner, log_queue: Queue = None):
        self.reckoner = reckoner
        self.log_queue = log_queue
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
        start = monotonic()
        try:
            state = self.reckoner.step(item)
        except Exception:
            if self.logger is not None:
                self.logger.exception("Reconstruction step failed at t=%s", item.t)
            raise
        elapsed = monotonic() - start

        if self.logger is not None:
            self.logger.info("step t=%.6f processed in %.6fs", item.t, elapsed)
            if self._prev_t is not None:
                dt = item.t - self._prev_t
                if elapsed > dt:
                    self.logger.warning(
                        "step t=%.6f took %.6fs, exceeding the %.6fs sample interval - "
                        "falling behind real-time",
                        item.t,
                        elapsed,
                        dt,
                    )
        self._prev_t = item.t

        return [str(value) for value in trajectory_state_to_row(state)]
