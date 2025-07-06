from datetime import datetime


class Price:
    def __init__(
        self,
        stock_code: str,
        market_code: str,
        open_: float,
        close: float,
        high: float,
        low: float,
        volume: float,
        time: int,
        frequency: str,
    ):
        self.stock_code = stock_code
        self.market_code = market_code
        self.open = open_
        self.close = close
        self.high = high
        self.low = low
        self.volume = volume
        self.time = time
        self.frequency = frequency

    def price_difference(self):
        return self.close - self.open

    def price_range(self):
        return self.high - self.low

    def price_volatility(self):
        if self.open != 0:
            return (self.high - self.low) / self.open
        return None

    def is_bullish(self):
        return self.close > self.open

    def is_bearish(self):
        return self.close < self.open

    def __eq__(self, other):
        if isinstance(other, Price):
            return (
                self.stock_code == other.stock_code
                and self.market_code == other.market_code
                and self.open == other.open
                and self.close == other.close
                and self.high == other.high
                and self.low == other.low
                and self.volume == other.volume
                and self.time == other.time
                and self.frequency == other.frequency
            )
        return False

    def __lt__(self, other):
        if isinstance(other, Price):
            return self.time < other.time
        return False
