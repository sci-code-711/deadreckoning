from multiprocessing import Queue

from deadrec.core import Core


class _FakeConnector:
    def __init__(self):
        self.started = False
        self.join_calls = []

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
