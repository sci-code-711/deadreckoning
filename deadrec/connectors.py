import csv
import re
import sqlite3
import time
from multiprocessing import Queue
from typing import List

from .io import imu_sample_from_row
from .runners import Runner, TerminateSignal
from .samples import ImuSample

_IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _validate_identifier(name: str, kind: str) -> str:
    """
    Validate that ``name`` is safe to interpolate into a SQL statement as an
    identifier (table/column name). Identifiers can't be passed as
    parameterised query arguments, so this guards against SQL injection via
    a crafted table or column name.

    """
    if not _IDENTIFIER_RE.match(name):
        raise ValueError(
            f"Invalid {kind} name {name!r}: must start with a letter or underscore and "
            "contain only letters, digits and underscores"
        )
    return name


class Connector(Runner):
    pass


class IngestConnector(Connector):
    def __init__(self):
        self.output_stream = Queue()
        super().__init__()


class OutputConnector(Connector):
    def __init__(self):
        self.input_stream = Queue()
        super().__init__()


class FromCSV(IngestConnector):
    def __init__(self, file_handle: str, sep: str = ",", skip_header: bool = True):
        """
        Args:
            * file_handle {``str``} -- Path to the CSV file to read.
            * sep {``str``} -- Column delimiter. Defaults to ``,``.
            * skip_header {``bool``} -- If ``True`` (the default), the
              first line of the file is consumed and discarded rather than
              pushed onto ``output_stream`` as a data row. Defaults to
              ``True`` since real CSVs almost always have a header row.

        """
        self.file_handle = file_handle
        self.delimiter = sep
        self.skip_header = skip_header
        super().__init__()

    def run(self):
        with open(self.file_handle) as csv_file:
            if self.skip_header:
                next(csv_file, None)

            for line in csv_file:
                row = line.rstrip().split(self.delimiter)
                self.output_stream.put(row)

        print("Completed reading CSV file")
        self.output_stream.put(TerminateSignal(True, None))

        return True


class LiveCSVReplay(IngestConnector):
    """
    Simulates a live IMU feed by replaying a CSV file (in the format read by
    :func:`deadrec.io.read_imu_csv`) as a stream of
    :class:`deadrec.samples.ImuSample`, paced to real wall-clock time
    according to each sample's own timestamp - there's no real phone to
    stream from yet, so this is how the streaming pipeline is proven to
    keep up with real-time sample arrival rather than just being fast in
    isolation.

    """

    def __init__(
        self,
        path: str,
        *,
        time_divisor: float = 1.0,
        speed: float = 1.0,
        sleep_fn=time.sleep,
    ):
        """
        Args:
            * path {``str``} -- Path to the CSV file to replay.
            * time_divisor {``float``} -- Every sample's timestamp is
              divided by this before use - e.g. 1000 if the file's
              timestamps are in milliseconds (as in
              ``example_data/Example_data.csv``). Must match whatever
              divisor the reconstruction downstream uses, since this same
              (divided) timestamp is what pacing is computed from - the
              pacing and the kinematics must agree on time units. Defaults
              to 1 (no conversion).
            * speed {``float``} -- Replay speed multiplier: 1.0 (the
              default) paces samples at real wall-clock time, higher values
              replay faster.
            * sleep_fn -- Called with the number of seconds to wait between
              consecutive samples. Defaults to :func:`time.sleep`; tests can
              inject a fake to assert computed durations without waiting.

        """
        self.path = path
        self.time_divisor = time_divisor
        self.speed = speed
        self.sleep_fn = sleep_fn
        super().__init__()

    def run(self):
        prev_t = None

        with open(self.path, newline="") as csv_file:
            for row in csv.DictReader(csv_file):
                sample = imu_sample_from_row(row)
                sample = ImuSample(
                    t=sample.t / self.time_divisor, accel=sample.accel, gyro=sample.gyro
                )

                if prev_t is not None:
                    self.sleep_fn(max(0.0, (sample.t - prev_t) / self.speed))
                prev_t = sample.t

                self.output_stream.put(sample)

        print("Completed live CSV replay")
        self.output_stream.put(TerminateSignal(True, None))

        return True


class ToCSV(OutputConnector):
    def __init__(self, file_handle: str, sep: str = ",", header: List[str] = None):
        """
        Args:
            * file_handle {``str``} -- Path to write the CSV file to.
            * sep {``str``} -- Column delimiter. Defaults to ``,``.
            * header {``List[str]``} -- If given, written as the first line
              of the file before any rows. Defaults to ``None`` (no header
              line), matching prior behaviour.

        """
        self.file_handle = file_handle
        self.delimiter = sep
        self.header = header
        super().__init__()

    def run(self):
        with open(self.file_handle, "w") as csv_file:
            if self.header is not None:
                csv_file.write(self.delimiter.join(self.header) + "\n")

            while True:
                item = self.input_stream.get()
                if isinstance(item, TerminateSignal):
                    # .get() returns None for a clean stop, or raises the
                    # original exception for a failure - either way this
                    # stops the loop without writing further rows; the
                    # `with` block still closes the file on the way out.
                    item.get()
                    break

                parsed_row = self.delimiter.join(item)
                csv_file.write(parsed_row + "\n")

        print("Completed CSV file write")


class ToSQLite(OutputConnector):
    def __init__(self, db_path: str, table_name: str, columns: List[str]):
        """
        Args:
            * db_path {``str``} -- Path to the SQLite database file to
              write to. It will be created if it doesn't already exist.
            * table_name {``str``} -- The name of the table to write rows
              to. It will be created (with all-TEXT columns) if it doesn't
              already exist.
            * columns {``List[str]``} -- The names of the columns to write
              each row's values to, in order.

        """
        self.db_path = db_path
        self.table_name = _validate_identifier(table_name, "table")
        self.columns = [_validate_identifier(column, "column") for column in columns]
        super().__init__()

    def run(self):
        connection = sqlite3.connect(self.db_path)
        count = 0
        try:
            column_defs = ", ".join(f'"{column}" TEXT' for column in self.columns)
            connection.execute(f'CREATE TABLE IF NOT EXISTS "{self.table_name}" ({column_defs})')

            placeholders = ", ".join("?" for _ in self.columns)
            insert_sql = f'INSERT INTO "{self.table_name}" VALUES ({placeholders})'

            while True:
                item = self.input_stream.get()
                if isinstance(item, TerminateSignal):
                    if not item.success:
                        connection.rollback()
                    item.get()
                    break

                connection.execute(insert_sql, tuple(item))
                count += 1

            connection.commit()
        finally:
            connection.close()

        print(f"Completed database write of {count} rows")
