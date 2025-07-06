import rclpy
from time import sleep
from .bt import Backtesting
from ..utils import get_ros_param
from ..data_provider import RealMarketDP


class RealMarketBacktesting(Backtesting):
    def init(self):
        super().init()
        self.duration = get_ros_param(self, "duration", 600.0)  # unit: second
        self.data_provider = RealMarketDP(self)
        self.data_provider.register_callback(self.handle_tick_data)

    def run_backtesting(self):
        sleep(self.duration)


def main(args=None):
    rclpy.init(args=args)
    RealMarketBacktesting.run_node("real_market_backtesting")


if __name__ == "__main__":
    main()
