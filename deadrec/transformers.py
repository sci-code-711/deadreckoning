from .runners import Runner, TerminateSignal
from multiprocessing import Queue
from abc import ABC, abstractmethod

class TransformerBase(Runner, ABC):
    def __init__(self, i_stream: Queue, o_stream: Queue):
        self.i_stream = i_stream
        self.o_stream = o_stream
        super().__init__()

    def run(self):
        count = 0
        while True:
            item = self.i_stream.get()
            if type(item) == TerminateSignal:
                self.o_stream.put(item)
                break

            count += 1
            self.o_stream.put(self.transformation(item))

        print(f"Transformer has finished processing {count} items")

        return True

    @abstractmethod
    def transformation(item):
        pass
