from deadrec.connectors import ToSQLite
from deadrec.runners import TerminateSignal
import sqlite3
import pytest


def test_to_sqlite_writes_rows(tmp_path):
    db_path = tmp_path / "output.db"
    connector = ToSQLite(str(db_path), "readings", ["timestamp", "value"])
    connector.input_stream.put(["1", "0.5"])
    connector.input_stream.put(["2", "0.75"])
    connector.input_stream.put(TerminateSignal(True, None))

    connector.run()

    connection = sqlite3.connect(db_path)
    rows = connection.execute("SELECT timestamp, value FROM readings").fetchall()
    connection.close()

    assert rows == [("1", "0.5"), ("2", "0.75")]


def test_to_sqlite_creates_table_if_missing(tmp_path):
    db_path = tmp_path / "output.db"
    connector = ToSQLite(str(db_path), "readings", ["a"])
    connector.input_stream.put(TerminateSignal(True, None))

    connector.run()

    connection = sqlite3.connect(db_path)
    tables = connection.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='readings'"
    ).fetchall()
    connection.close()

    assert tables == [("readings",)]


def test_to_sqlite_appends_to_existing_table(tmp_path):
    db_path = tmp_path / "output.db"

    first = ToSQLite(str(db_path), "readings", ["a"])
    first.input_stream.put(["1"])
    first.input_stream.put(TerminateSignal(True, None))
    first.run()

    second = ToSQLite(str(db_path), "readings", ["a"])
    second.input_stream.put(["2"])
    second.input_stream.put(TerminateSignal(True, None))
    second.run()

    connection = sqlite3.connect(db_path)
    rows = connection.execute("SELECT a FROM readings").fetchall()
    connection.close()

    assert rows == [("1",), ("2",)]


@pytest.mark.parametrize("bad_name", ["bad name", "1table", "table;DROP TABLE x", ""])
def test_to_sqlite_rejects_unsafe_table_name(bad_name, tmp_path):
    with pytest.raises(ValueError):
        ToSQLite(str(tmp_path / "output.db"), bad_name, ["a"])


@pytest.mark.parametrize("bad_name", ["bad name", "1col", "col;DROP TABLE x"])
def test_to_sqlite_rejects_unsafe_column_name(bad_name, tmp_path):
    with pytest.raises(ValueError):
        ToSQLite(str(tmp_path / "output.db"), "readings", [bad_name])
