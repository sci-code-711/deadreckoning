from deadrec.logger import QueueLogListener, get_worker_logger
from multiprocessing import Process, Queue
import logging


def _log_from_worker(log_queue: Queue, message: str):
    logger = get_worker_logger(log_queue, "worker")
    logger.info(message)


def test_get_worker_logger_sends_records_to_queue():
    log_queue = Queue()
    logger = get_worker_logger(log_queue, "test_get_worker_logger_sends_records_to_queue")

    logger.info("hello")

    record = log_queue.get(timeout=5)
    assert record.getMessage() == "hello"
    assert record.name == "test_get_worker_logger_sends_records_to_queue"


def test_get_worker_logger_does_not_propagate_to_root():
    log_queue = Queue()
    logger = get_worker_logger(log_queue, "test_get_worker_logger_does_not_propagate_to_root")

    assert logger.propagate is False
    assert len(logger.handlers) == 1


def test_queue_log_listener_dispatches_to_handler():
    log_queue = Queue()
    records = []

    class ListHandler(logging.Handler):
        def emit(self, record):
            records.append(record.getMessage())

    listener = QueueLogListener(log_queue, ListHandler())
    listener.start()
    try:
        logger = get_worker_logger(log_queue, "test_queue_log_listener_dispatches_to_handler")
        logger.info("from same process")
    finally:
        listener.stop()

    assert "from same process" in records


def test_queue_log_listener_collects_records_from_multiple_processes():
    log_queue = Queue()
    records = []

    class ListHandler(logging.Handler):
        def emit(self, record):
            records.append(record.getMessage())

    listener = QueueLogListener(log_queue, ListHandler())
    listener.start()
    try:
        processes = [
            Process(target=_log_from_worker, args=(log_queue, f"message {i}")) for i in range(3)
        ]
        for process in processes:
            process.start()
        for process in processes:
            process.join(timeout=10)
    finally:
        listener.stop()

    assert sorted(records) == ["message 0", "message 1", "message 2"]
