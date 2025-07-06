class OrderTask:
    def __init__(self, pending=True, is_successful=True, msg=""):
        self._pending = pending
        self._is_successful = is_successful
        self._msg = msg

    def add_msg(self, msg: str):
        self._msg = msg

    def get_msg(self):
        return self._msg

    def is_successful(self):
        if self._pending:
            return False
        return self._is_successful

    def complete(self, result=True):
        self._pending = False
        self._is_successful = result

    def is_complete(self):
        return not self._pending
