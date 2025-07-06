import numpy as np
from system_interface.msg import Order
from .agent import Agent

class CombinedSignalAgent(Agent):
    type = "CombinedSignal"

    def __init__(self,
                 rsi_window=14, rsi_overbought=70, rsi_oversold=30,
                 macd_short_window=12, macd_long_window=26, macd_signal_window=9,
                 position_size=100.0):
        self.rsi_window = rsi_window
        self.rsi_overbought = rsi_overbought
        self.rsi_oversold = rsi_oversold
        self.rsi_prices = []
        self.rsi_values = []

        self.macd_short_window = macd_short_window
        self.macd_long_window = macd_long_window
        self.macd_signal_window = macd_signal_window
        self.macd_prices = []
        self.macd_macd_line = []
        self.macd_signal_line = []
        self.macd_histogram = []

        self.position_size = position_size

    def calculate_rsi(self):
        if len(self.rsi_prices) < self.rsi_window + 1:
            return None

        prices = np.array(self.rsi_prices)
        deltas = np.diff(prices)
        seed = deltas[:self.rsi_window]
        up = seed[seed >= 0].sum() / self.rsi_window
        down = -seed[seed < 0].sum() / self.rsi_window
        rs = up / down if down != 0 else 0
        rsi = 100.0 - (100.0 / (1.0 + rs))
        rsi_values = [rsi]

        for delta in deltas[self.rsi_window:]:
            if delta > 0:
                upval = delta
                downval = 0.0
            else:
                upval = 0.0
                downval = -delta

            up = (up * (self.rsi_window - 1) + upval) / self.rsi_window
            down = (down * (self.rsi_window - 1) + downval) / self.rsi_window
            rs = up / down if down != 0 else 0
            rsi = 100.0 - (100.0 / (1.0 + rs))
            rsi_values.append(rsi)

        return rsi_values[-1]

    def ema(self, prices, window):
        if len(prices) < window:
            return []
        ema_values = []
        alpha = 2 / (window + 1)
        ema_prev = np.mean(prices[:window])
        ema_values.append(ema_prev)
        for price in prices[window:]:
            ema_current = alpha * price + (1 - alpha) * ema_prev
            ema_values.append(ema_current)
            ema_prev = ema_current
        return ema_values

    def calculate_macd(self):
        if len(self.macd_prices) < self.macd_long_window:
            return None, None, None

        short_ema = self.ema(self.macd_prices, self.macd_short_window)
        long_ema = self.ema(self.macd_prices, self.macd_long_window)

        if not short_ema or not long_ema or len(short_ema) < len(long_ema):
            return None, None, None

        short_ema = short_ema[-len(long_ema):]

        macd_line = np.array(short_ema) - np.array(long_ema)

        signal_line = self.ema(macd_line, self.macd_signal_window)

        if not signal_line or len(signal_line) < len(macd_line):
            return None, None, None

        macd_line = macd_line[-len(signal_line):]
        histogram = macd_line - np.array(signal_line)

        return macd_line, signal_line, histogram

    def on_price(self, price):
        """
        Callback from input_topic
        """
        self.rsi_prices.append(price.price)
        self.macd_prices.append(price.price)

        # RSI
        rsi_value = self.calculate_rsi()
        rsi_signal = None
        if rsi_value is not None:
            self.rsi_values.append(rsi_value)
            print(f"RSI Value: {rsi_value:.2f}")
            if rsi_value > self.rsi_overbought:
                rsi_signal = 'sell'
            elif rsi_value < self.rsi_oversold:
                rsi_signal = 'buy'

        # MACD
        macd_line, signal_line, histogram = self.calculate_macd()
        macd_signal = None
        if macd_line is not None and signal_line is not None and len(histogram) >= 2:
            self.macd_macd_line.append(macd_line[-1])
            self.macd_signal_line.append(signal_line[-1])
            self.macd_histogram.append(histogram[-1])
            print(f"MACD Line: {macd_line[-1]:.2f}, Signal Line: {signal_line[-1]:.2f}, Histogram: {histogram[-1]:.2f}")

            if histogram[-1] > 0 and histogram[-2] <= 0:
                macd_signal = 'buy'
            elif histogram[-1] < 0 and histogram[-2] >= 0:
                macd_signal = 'sell'

        if rsi_signal == 'buy' and macd_signal == 'buy':
            order = Order()
            order.symbol = price.symbol
            order.exchange = price.exchange
            order.price = price.price
            order.size = self.position_size / price.price
            order.time = price.time
            order.side = True
            return order
        elif rsi_signal == 'sell' and macd_signal == 'sell':
            order = Order()
            order.symbol = price.symbol
            order.exchange = price.exchange
            order.price = price.price
            order.size = self.position_size / price.price
            order.time = price.time
            order.side = False
            return order
        else:
            return None
