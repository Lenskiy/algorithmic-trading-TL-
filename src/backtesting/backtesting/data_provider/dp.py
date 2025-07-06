from system_interface.msg import TickData


class DataProvider:
    def get_data(self) -> TickData:
        raise NotImplementedError

    def register_callback(self, callback):
        raise NotImplementedError

    def can_continue(self) -> bool:
        raise NotImplementedError
