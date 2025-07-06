import rclpy
from .bt import Backtesting
from ..data_provider import LocalFileDP
from ..utils import get_ros_param


class LocalFileBacktesting(Backtesting):
    def init(self):
        super().init()
        filepath = get_ros_param(
            self, "filepath", "src/trading_strategies/data/sp500_adj_close_prices.csv"
        )
        data_num = get_ros_param(self, "data_num", 1000)
        time_gap = get_ros_param(self, "time_gap", 0.01)  # unit: second
        self.data_provider = LocalFileDP(
            filepath, self.symbol, self.exchange, data_num, time_gap
        )


def main(args=None):
    rclpy.init(args=args)
    LocalFileBacktesting.run_node("local_file_backtesting")


if __name__ == "__main__":
    main()
