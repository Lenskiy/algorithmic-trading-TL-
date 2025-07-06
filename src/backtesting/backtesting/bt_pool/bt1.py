import rclpy
from .bt import Backtesting
from ..data_provider import HistoricalDataDP
from ..utils import get_ros_param


class HistoricalDataBacktesting(Backtesting):
    def init(self):
        super().init()
        self.start_time = get_ros_param(self, "start_time", 1726012266_000)
        self.end_time = get_ros_param(self, "end_time", self.start_time + 60_000)
        time_gap = get_ros_param(self, "time_gap", 0.01)  # unit: second
        self.data_provider = HistoricalDataDP(
            self.symbol, self.exchange, self.start_time, self.end_time, self, time_gap
        )


def main(args=None):
    rclpy.init(args=args)
    HistoricalDataBacktesting.run_node("historical_data_backtesting")


if __name__ == "__main__":
    main()
