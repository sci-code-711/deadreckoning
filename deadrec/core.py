from multiprocessing import Process
from .transformers import TransformerBase
from typing import List


class PipelineError(RuntimeError):
    """Raised by Core.terminate() when a connector or transformer process
    exited with a non-zero exit code, so a pipeline failure is visible to
    the caller instead of being silently swallowed."""


class DefaultTransformer(TransformerBase):
    def transformation(self, item):
        return item


class Core:
    def __init__(self, name: str):
        self.name = name
        self.transformer = DefaultTransformer
        self.transformer_kwargs: dict = {}
        self.connectors: List[Process] = []

    def set_input_connector(self, connector: Process):
        self.i_connector = connector
        self.i_stream = self.i_connector.output_stream
        self.connectors.append(connector)

    def set_output_connector(self, connector: Process):
        self.o_connector = connector
        self.o_stream = self.o_connector.input_stream
        self.connectors.append(connector)

    def launch(self):
        self.transformer_instance = self.transformer(
            self.i_stream, self.o_stream, **self.transformer_kwargs
        )
        for connector in self.connectors[::-1]:
            connector.start()
        self.transformer_instance.start()

    def terminate(self, force=False, timeout=None):
        self.i_connector.join(timeout)
        self.transformer_instance.join(timeout)
        self.o_connector.join(timeout)

        failures = [
            (name, process.exitcode)
            for name, process in (
                ("input connector", self.i_connector),
                ("transformer", self.transformer_instance),
                ("output connector", self.o_connector),
            )
            if process.exitcode not in (None, 0)
        ]
        if failures:
            summary = ", ".join(f"{name} exited with code {code}" for name, code in failures)
            raise PipelineError(f"Pipeline {self.name!r} failed: {summary}")
