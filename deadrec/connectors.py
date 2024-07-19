from multiprocessing import Queue
from .runners import Runner, TerminateSignal

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

        print(f"Completed reading CSV file")
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
                if type(item) == TerminateSignal:
                    break

                parsed_row = self.delimiter.join(item)
                csv_file.write(parsed_row + "\n")

        print(f"Completed CSV file write")
