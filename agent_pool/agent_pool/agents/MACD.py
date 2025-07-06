import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from .agent import Agent
from ..mixture import Mixture
from ..utils import create_order, get_ros_param


def calculate_macd(prices, short_window=12, long_window=26, signal_window=9):
    assert len(prices) >= long_window, "the amount of prices is not enough"

    # 计算短期和长期 EMA
    short_ema = ema(prices, short_window)
    long_ema = ema(prices, long_window)

    # MACD 线是短期 EMA 减去长期 EMA
    macd_line = short_ema - long_ema

    # 信号线是 MACD 线的信号窗口的 EMA
    signal_line = ema(macd_line, signal_window)

    # 柱状图是 MACD 线减去信号线
    histogram = macd_line - signal_line

    return macd_line, signal_line, histogram


def ema(prices, window):
    ema = np.zeros(len(prices))
    ema[0] = prices[0]
    alpha = 2 / (window + 1)
    for i in range(1, len(prices)):
        ema[i] = alpha * prices[i] + (1 - alpha) * ema[i - 1]
    return ema


def generate_signal(prices, short_window, long_window, signal_window):
    if len(prices) < long_window:
        return "hold"

    macd_line, signal_line, histogram = calculate_macd(
        prices, short_window, long_window, signal_window
    )
    print(macd_line[-2:], signal_line[-2:], histogram[-2:])

    # 简单的买卖信号生成逻辑：
    # 如果柱状图从负变正，返回买入信号 'buy'
    if histogram[-1] > 0 and histogram[-2] <= 0:
        return "buy"

    # 如果柱状图从正变负，返回卖出信号 'sell'
    if histogram[-1] < 0 and histogram[-2] >= 0:
        return "sell"

    return "hold"  # 没有交易信号时返回 'hold'


class MACDAgent(Agent):
    def init(self):
        super().init()

        enable_mix = get_ros_param(self, "mix", True)
        self.databin = Mixture(self, enable=enable_mix)

        self.short_window = 12
        self.long_window = 26
        self.signal_window = 9
        self.position_size = 100.0

    def on_price(self, tick_data):
        self.databin.append(tick_data)
        prices = [d.price for d in self.databin]
        signal = generate_signal(
            prices,
            self.short_window,
            self.long_window,
            self.signal_window,
        )
        return create_order(tick_data, self.position_size, signal)


def main(args=None):
    rclpy.init(args=args)
    node = MACDAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
