import numpy as np
import rclpy
from .agent import Agent
from ..mixture import Mixture
from ..utils import create_order, get_ros_param


def calculate_bollinger_bands(prices, window=20, num_std=2):
    assert len(prices) >= window, "the amount of prices is not enough"

    # 取最后window个价格
    prices_array = np.array(prices[-window:])

    # 计算简单移动平均线（SMA）和标准差
    sma = np.mean(prices_array)
    std = np.std(prices_array)

    # 计算上轨线和下轨线
    upper_band = sma + num_std * std
    lower_band = sma - num_std * std

    return sma, upper_band, lower_band


def generate_signal(prices, window=20, num_std=2):
    if len(prices) < window:
        return "hold"

    sma, upper_band, lower_band = calculate_bollinger_bands(prices, window, num_std)

    print(f"SMA: {sma:.2f}, Upper Band: {upper_band:.2f}, Lower Band: {lower_band:.2f}")

    # 买入信号：价格低于下轨线
    if prices[-1] < lower_band:
        return "buy"

    # 卖出信号：价格高于上轨线
    elif prices[-1] > upper_band:
        return "sell"

    return "hold"


class BollingerBandsAgent(Agent):
    def init(self):
        super().init()

        enable_mix = get_ros_param(self, "mix", True)
        self.databin = Mixture(self, enable=enable_mix)

        self.window = 20
        self.num_std = 2
        self.position_size = 100.0

    def on_price(self, tick_data):
        # 使用 Mixture 管理数据
        self.databin.append(tick_data)
        prices = [d.price for d in self.databin]
        signal = generate_signal(prices, self.window, self.num_std)
        return create_order(tick_data, self.position_size, signal)


def main(args=None):
    rclpy.init(args=args)
    node = BollingerBandsAgent()
    rclpy.spin(node)
