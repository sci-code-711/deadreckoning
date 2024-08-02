from multiprocessing import Queue
from .runners import Runner, TerminateSignal
from pymongo import MongoClient
from datetime import datetime, timedelta

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
    def __init__(self, file_handle: str, sep: str = ",", header=True):
        self.file_handle = file_handle
        self.delimiter = sep
        self.header = header
        super().__init__()

    def run(self):
        with open(self.file_handle) as csv_file:

            if self.header:
                headers = next(csv_file).rstrip().split(self.delimiter)

            for line in csv_file:
                row = line.rstrip().split(self.delimiter)
                processed_row = {
                    key: value for key, value in zip(headers, row)
                }
                self.output_stream.put(processed_row)

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


class ToMongo(OutputConnector):
    def __init__(
            self,
            url: str,
            database: str,
            collection: str,
            mapper: callable
        ):
        self.url = url
        self.database_name = database
        self.collection_name = collection
        self.mapper = mapper
        super().__init__()

    def run(self):
        self.mongodb_client = MongoClient(self.url)

        self.collection = self.mongodb_client[
            self.database_name
        ].get_collection(
            self.collection_name
        )

        while True:
            item = self.input_stream.get()
            if isinstance(item, TerminateSignal):
                break

            document = self.mapper(item)

            self.collection.insert_one(document)
