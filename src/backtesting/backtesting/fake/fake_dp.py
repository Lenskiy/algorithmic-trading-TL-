from random import randint
import rclpy
from rclpy.node import Node
from system_interface.msg import TickData
from system_interface.srv import GetHistoricalTickDatas
from ..utils import ServiceServer


class FakeDP(Node):
    def __init__(self):
        super().__init__("fake_data_provider")
        self.get_historical_tick_datas_server = ServiceServer(
            self,
            GetHistoricalTickDatas,
            "get_historical_tick_datas",
            self.get_historical_tick_datas,
        )

    def get_historical_tick_datas(
        self, symbol: str, exchange: str, start_time: int, end_time: int
    ):
        print("get_historical_tick_datas", symbol, exchange, start_time, end_time)
        tick_datas = []
        price = randint(110, 120) * 1.0
        d = 0

        for time in range(start_time, end_time, 1_000):
            tick_data = TickData()
            tick_data.symbol = symbol
            tick_data.exchange = exchange
            tick_data.size = 100.0
            tick_data.is_buy = True
            tick_data.time = time
            if randint(0, 6) == 0:
                d = (d + 1) % 2

            if d:
                price = price * randint(100, 101) / 100.0
            else:
                price = price * randint(99, 100) / 100.0

            tick_data.price = price
            tick_datas.append(tick_data)

        return tick_datas


def main(args=None):
    rclpy.init(args=args)
    node = FakeDP()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
