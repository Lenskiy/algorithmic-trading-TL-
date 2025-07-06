from collections import deque
import rclpy
from .agent import Agent
from ..mixture import Mixture
from ..utils import create_order, get_ros_param


def calculate_single_rsi(prices, window):
    assert len(prices) >= window, "the amount of prices is not enough"

    # 取prices列表最后一个window大小的子集
    recent_prices = prices[-window:]

    gains = []
    losses = []

    # 遍历子集，计算每一天的价格变化
    for i in range(1, len(recent_prices)):
        change = recent_prices[i] - recent_prices[i - 1]
        if change > 0:
            gains.append(change)
        else:
            losses.append(abs(change))

    # 计算平均上涨和下跌
    avg_gain = sum(gains) / window
    avg_loss = sum(losses) / window if losses else 0.0  # 避免除以0

    # 计算RS（相对强弱）
    rs = avg_gain / avg_loss if avg_loss != 0 else 0.0

    # 计算RSI
    rsi = 100 - (100 / (1 + rs))
    return rsi


def generate_signal(prices, window, overbought, oversold):
    if len(prices) < window:
        return "hold"

    rsi = calculate_single_rsi(prices, window)
    print(f"RSI: {rsi}")

    if rsi > overbought:
        return "sell"
    if rsi < oversold:
        return "buy"
    return "hold"


class RSIAgent(Agent):
    def init(self):
        super().init()

        enable_mix = get_ros_param(self, "mix", True)
        self.databin = Mixture(self, enable=enable_mix)

        self.window = 14
        self.overbought = 70
        self.oversold = 30
        self.position_size = 100.0

    def on_price(self, tick_data):
        self.databin.append(tick_data)
        prices = [d.price for d in self.databin]
        signal = generate_signal(prices, self.window, self.overbought, self.oversold)
        return create_order(tick_data, self.position_size, signal)


def main(args=None):
    rclpy.init(args=args)
    node = RSIAgent()
    rclpy.spin(node)
