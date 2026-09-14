"""
A module providing logging utilities that are safe to use across multiple
processes.

The connectors, transformers, and runners in this package each run in their
own ``multiprocessing.Process``. Attaching a regular ``logging.Handler``
(e.g. one that writes to a file or stream) directly in each process is
unsafe, since concurrent writes from separate processes can interleave and
corrupt the output. Instead, worker processes should log through
``get_worker_logger``, which routes records over a ``multiprocessing.Queue``
to a single ``QueueLogListener`` running in the main process, which then
handles them one at a time.

"""

import logging
import logging.handlers
from multiprocessing import Queue

DEFAULT_LOG_FORMAT = "%(asctime)s [%(processName)s] %(levelname)s %(name)s: %(message)s"


def get_worker_logger(
    log_queue: Queue, name: str = None, level: int = logging.INFO
) -> logging.Logger:
    """
    Build a logger for use within a worker process.

    Args:
        * log_queue {``multiprocessing.Queue``} -- The queue shared with a
          ``QueueLogListener`` that log records should be sent over.
        * name {``str``} -- The name of the logger to create, passed
          through to :func:`logging.getLogger`. Defaults to the root
          logger.
        * level {``int``} -- The minimum level of messages that will be
          handled. Defaults to ``logging.INFO``.

    Returns:
        * {``logging.Logger``} -- A logger which sends all records to
          ``log_queue`` instead of handling them directly.

    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    return logger


class QueueLogListener(logging.handlers.QueueListener):
    """
    A ``QueueListener`` that collects log records sent by worker processes
    (via loggers created with :func:`get_worker_logger`) over a
    ``multiprocessing.Queue``, and dispatches them to the given handlers
    from a single dedicated thread in the listening process.

    This should be constructed and started in the main process before any
    worker processes are launched, and stopped once they have all
    finished.

    """

    def __init__(self, log_queue: Queue, *handlers: logging.Handler):
        """
        Args:
            * log_queue {``multiprocessing.Queue``} -- The queue that
              worker processes will send log records over.
            * handlers {``logging.Handler``} -- The handlers that received
              log records should be dispatched to. If none are given, a
              single ``StreamHandler`` using ``DEFAULT_LOG_FORMAT`` is
              used.

        """
        if not handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter(DEFAULT_LOG_FORMAT))
            handlers = (handler,)

        super().__init__(log_queue, *handlers, respect_handler_level=True)
