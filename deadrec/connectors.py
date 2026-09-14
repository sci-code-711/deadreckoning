import re
import sqlite3
from multiprocessing import Queue
from typing import List
from .runners import Runner, TerminateSignal

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
    def __init__(self, file_handle: str, sep: str = ","):
        self.file_handle = file_handle
        self.delimiter = sep
        super().__init__()

    def run(self):
        with open(self.file_handle) as csv_file:
            for line in csv_file:
                row = line.rstrip().split(self.delimiter)
                self.output_stream.put(row)

        print("Completed reading CSV file")
        self.output_stream.put(TerminateSignal(True, None))

        return True


class ToCSV(OutputConnector):
    def __init__(self, file_handle: str, sep: str = ","):
        self.file_handle = file_handle
        self.delimiter = sep
        super().__init__()

    def run(self):
        with open(self.file_handle, "w") as csv_file:
            while True:
                item = self.input_stream.get()
                if isinstance(item, TerminateSignal):
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
                    break

                connection.execute(insert_sql, tuple(item))
                count += 1

            connection.commit()
        finally:
            connection.close()

        print(f"Completed database write of {count} rows")
