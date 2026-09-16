from deadrec.connectors import FromCSV, LiveCSVReplay, ToSQLite
from deadrec.runners import TerminateSignal
from deadrec.samples import ImuSample
import sqlite3
import pytest


def _drain(connector):
    rows = []
    while True:
        item = connector.output_stream.get()
        if isinstance(item, TerminateSignal):
            return rows, item
        rows.append(item)


def test_from_csv_skips_header_by_default(tmp_path):
    csv_path = tmp_path / "input.csv"
    csv_path.write_text("t,ax,ay,az\n1,0.1,0.2,0.3\n2,0.4,0.5,0.6\n")

    connector = FromCSV(str(csv_path))
    connector.run()

    rows, terminate_signal = _drain(connector)

    assert rows == [["1", "0.1", "0.2", "0.3"], ["2", "0.4", "0.5", "0.6"]]
    assert terminate_signal.success is True


def test_from_csv_skip_header_false_includes_first_line(tmp_path):
    csv_path = tmp_path / "input.csv"
    csv_path.write_text("t,ax,ay,az\n1,0.1,0.2,0.3\n")

    connector = FromCSV(str(csv_path), skip_header=False)
    connector.run()

    rows, _ = _drain(connector)

    assert rows == [["t", "ax", "ay", "az"], ["1", "0.1", "0.2", "0.3"]]


def _drain_samples(connector):
    samples = []
    while True:
        item = connector.output_stream.get()
        if isinstance(item, TerminateSignal):
            return samples, item
        samples.append(item)


def test_live_csv_replay_paces_between_samples_using_sleep_fn(tmp_path):
    csv_path = tmp_path / "input.csv"
    csv_path.write_text(
        "t,ax,ay,az,vl,vm,vn\n0,0.1,0.2,0.3,1,2,3\n1000,0.4,0.5,0.6,4,5,6\n3000,0.7,0.8,0.9,7,8,9\n"
    )
    sleep_calls = []

    connector = LiveCSVReplay(
        str(csv_path), time_divisor=1000.0, speed=2.0, sleep_fn=sleep_calls.append
    )
    connector.run()

    samples, terminate_signal = _drain_samples(connector)

    assert [s.t for s in samples] == pytest.approx([0.0, 1.0, 3.0])
    assert all(isinstance(sample, ImuSample) for sample in samples)
    # Real dt after the /1000 divisor is 1.0s then 2.0s; /speed=2.0 halves both.
    assert sleep_calls == pytest.approx([0.5, 1.0])
    assert terminate_signal.success is True


def test_live_csv_replay_defaults_to_no_time_conversion_and_real_time_pacing(tmp_path):
    csv_path = tmp_path / "input.csv"
    csv_path.write_text("t,ax,ay,az,vl,vm,vn\n0,0.1,0.2,0.3,1,2,3\n0.5,0.4,0.5,0.6,4,5,6\n")
    sleep_calls = []

    connector = LiveCSVReplay(str(csv_path), sleep_fn=sleep_calls.append)
    connector.run()

    samples, _ = _drain_samples(connector)

    assert [s.t for s in samples] == pytest.approx([0.0, 0.5])
    assert sleep_calls == pytest.approx([0.5])


def test_live_csv_replay_never_sleeps_for_a_single_sample(tmp_path):
    csv_path = tmp_path / "input.csv"
    csv_path.write_text("t,ax,ay,az,vl,vm,vn\n0,0.1,0.2,0.3,1,2,3\n")
    sleep_calls = []

    connector = LiveCSVReplay(str(csv_path), sleep_fn=sleep_calls.append)
    connector.run()

    samples, _ = _drain_samples(connector)

    assert len(samples) == 1
    assert sleep_calls == []


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
