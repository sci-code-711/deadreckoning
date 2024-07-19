from multiprocessing import Process, Queue
from .transformers import TransformerBase
from typing import List

class DefaultTransformer(TransformerBase):
    def transformation(self, item):
        return item

class Core:
    def __init__(self, name: str):
        self.name = name
        self.transformer = DefaultTransformer
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
            self.i_stream, self.o_stream
        )
        for connector in self.connectors[::-1]:
            connector.start()
        self.transformer_instance.start()

    def terminate(self, force=False):
        self.i_connector.join()
        self.transformer_instance.join()
        self.o_connector.join()
