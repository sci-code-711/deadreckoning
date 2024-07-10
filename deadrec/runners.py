from multiprocessing import Process

class TerminateSignal:
    def __init__(
            self, success, payload, *, msg="Runner successfully completed"
        ):
        self.success = success
        self.msg = msg
        self.payload = payload

    def get(self):
        if self.success:
            return self.payload

        raise self.payload

class Runner(Process):
    pass
