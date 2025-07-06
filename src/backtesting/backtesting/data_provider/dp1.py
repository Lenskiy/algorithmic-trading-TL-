from time import sleep
from rclpy.node import Node
from system_interface.msg import TickData
from system_interface.srv import GetHistoricalTickDatas
from .dp import DataProvider
from ..utils import ServiceClient


class HistoricalDataDP(DataProvider):
    def __init__(
        self, symbol, exchange, start_time, end_time, node: Node, time_gap: float
    ):
        self.symbol = symbol
        self.exchange = exchange

        self.start_time = start_time
        self.current_time = start_time
        self.end_time = end_time
        self.time_gap = time_gap

        self.tick_datas: list[TickData] = []
        self.index = 0
        self.client = ServiceClient(
            node,
            GetHistoricalTickDatas,
            "get_historical_tick_datas",
        )

    def get_historical_tick_datas(self, symbol, exchange, start_time, end_time):
        print("get_historical_tick_datas", symbol, exchange, start_time, end_time)
        return self.client.call(symbol, exchange, start_time, end_time)

    def request_datas(self):
        next_time = min(self.current_time + 60_000, self.end_time)
        datas = self.get_historical_tick_datas(
            self.symbol,
            self.exchange,
            self.current_time,
            next_time,
        )
        self.tick_datas = datas  # type: ignore
        self.current_time = next_time

    def data_is_used_up(self):
        return self.index >= len(self.tick_datas)

    def time_is_over(self):
        return self.current_time >= self.end_time

    def get_data(self) -> TickData:
        # 重新获取数据
        if self.data_is_used_up():
            self.index = 0
            self.request_datas()

        sleep(self.time_gap)
        tick_data = self.tick_datas[self.index]
        self.index += 1
        return tick_data

    def can_continue(self) -> bool:
        if self.time_is_over() and self.data_is_used_up():
            return False
        return True
