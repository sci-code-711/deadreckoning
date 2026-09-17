from multiprocessing import Queue

import pytest

from deadrec.connectors import FromCSV, ToCSV
from deadrec.core import Core, PipelineError
from deadrec.transformers import TransformerBase


class _FakeConnector:
    def __init__(self):
        self.started = False
        self.join_calls = []
        self.exitcode = 0

    def start(self):
        self.started = True

    def join(self, timeout=None):
        self.join_calls.append(timeout)


class _FakeInputConnector(_FakeConnector):
    def __init__(self):
        super().__init__()
        self.output_stream = Queue()


class _FakeOutputConnector(_FakeConnector):
    def __init__(self):
        super().__init__()
        self.input_stream = Queue()


class _FakeTransformer:
    def __init__(self, i_stream, o_stream, **kwargs):
        self.i_stream = i_stream
        self.o_stream = o_stream
        self.kwargs = kwargs
        self.started = False
        self.join_calls = []
        self.exitcode = 0

    def start(self):
        self.started = True

    def join(self, timeout=None):
        self.join_calls.append(timeout)


def test_core_transformer_kwargs_defaults_to_empty_dict():
    core = Core("test")

    assert core.transformer_kwargs == {}


def test_core_launch_passes_transformer_kwargs_through():
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer
    core.transformer_kwargs = {"reckoner": "fake-reckoner", "log_queue": None}

    core.launch()

    assert core.transformer_instance.kwargs == {"reckoner": "fake-reckoner", "log_queue": None}
    assert core.transformer_instance.started is True
    assert core.i_connector.started is True
    assert core.o_connector.started is True


def test_core_launch_with_no_transformer_kwargs_constructs_with_only_streams():
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer

    core.launch()

    assert core.transformer_instance.kwargs == {}


def test_core_terminate_defaults_to_no_timeout():
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer
    core.launch()

    core.terminate()

    assert core.i_connector.join_calls == [None]
    assert core.transformer_instance.join_calls == [None]
    assert core.o_connector.join_calls == [None]


def test_core_terminate_passes_timeout_to_each_join():
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer
    core.launch()

    core.terminate(timeout=5)

    assert core.i_connector.join_calls == [5]
    assert core.transformer_instance.join_calls == [5]
    assert core.o_connector.join_calls == [5]


def test_core_terminate_raises_when_a_process_exits_nonzero():
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer
    core.launch()
    core.transformer_instance.exitcode = 1

    with pytest.raises(PipelineError, match="transformer exited with code 1"):
        core.terminate()


def test_core_terminate_does_not_raise_when_all_processes_exit_cleanly():
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer
    core.launch()

    core.terminate()  # should not raise


def test_core_terminate_does_not_raise_for_a_still_running_process():
    # exitcode is None while a process hasn't finished (e.g. join() timed
    # out) - that's not the same as a failure, so it must not raise.
    core = Core("test")
    core.set_input_connector(_FakeInputConnector())
    core.set_output_connector(_FakeOutputConnector())
    core.transformer = _FakeTransformer
    core.launch()
    core.transformer_instance.exitcode = None

    core.terminate()  # should not raise


class _FailingTransformer(TransformerBase):
    def transformation(self, item):
        raise ValueError(f"deliberate failure on {item!r}")


def test_core_terminate_raises_end_to_end_when_transformer_fails(tmp_path):
    # A real Core, with real multiprocessing.Process-based connectors, run
    # through a genuine failure - proves the fix end-to-end rather than
    # just at the unit level: the transformer's exception is forwarded as a
    # failure TerminateSignal (deadrec/transformers.py), ToCSV sees it and
    # raises instead of treating it as a clean stop (deadrec/connectors.py),
    # and Core.terminate() surfaces the resulting non-zero exit codes
    # instead of hanging or swallowing the failure (deadrec/core.py).
    in_path = tmp_path / "in.csv"
    in_path.write_text("a,b\n1,2\n3,4\n")
    out_path = tmp_path / "out.csv"

    core = Core("failing")
    core.set_input_connector(FromCSV(str(in_path)))
    core.set_output_connector(ToCSV(str(out_path)))
    core.transformer = _FailingTransformer
    core.launch()

    with pytest.raises(PipelineError):
        core.terminate(timeout=10)
