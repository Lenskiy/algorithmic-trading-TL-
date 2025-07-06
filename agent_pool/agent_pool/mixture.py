from rclpy.node import Node
from collections import deque
from system_interface.msg import TickData
from system_interface.srv import GetHistoricalTickDatas
from .utils import ServiceClient


class Mixture:
    """存储最近max_len个数据，并可以把历史数据和实时数据混合"""

    def __init__(self, node: Node, max_len=100, enable=True) -> None:
        self.enable = enable
        """是否启用mixture功能"""

        if enable:
            self.cli = ServiceClient(
                node,
                GetHistoricalTickDatas,
                "get_historical_tick_datas",
            )
            self.first = True
            self.max_len = max_len

        self.tick_datas: deque[TickData] = deque(maxlen=max_len)

    def call_historical_tick_datas(self, tick_data: TickData):
        future = self.cli.call_async(
            tick_data.symbol,
            tick_data.exchange,
            tick_data.time - 100_000,
            tick_data.time,
        )

        def func(_):
            print("receive historical data")
            tick_datas = future.result()
            assert tick_datas is not None
            tick_datas += list(self.tick_datas)

            # 这里只会取最后max_len个数据
            self.tick_datas = deque(tick_datas, self.max_len)

        future.add_done_callback(func)

    def append(self, tick_data: TickData):
        if self.enable and self.first:
            self.first = False
            self.call_historical_tick_datas(tick_data)

        self.tick_datas.append(tick_data)

    def __getitem__(self, index):
        return self.tick_datas[index]
