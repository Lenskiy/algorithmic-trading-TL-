import numpy as np
import pandas as pd


class Record:
    time: int
    cash: float
    stock_price: float
    stock_position: float

    def __init__(
        self,
        time: int,
        cash: float,
        stock_price: float,
        stock_position: float,
    ):
        self.time = time
        self.stock_price = stock_price
        self.stock_position = stock_position
        self.cash = cash

    @property
    def stock_value(self):
        return self.stock_price * self.stock_position

    @property
    def total_value(self):
        return self.stock_value + self.cash

    def __str__(self):
        return f"{self.stock_position}"

    def to_dict(self):
        return {
            "time": self.time,
            "stock_price": self.stock_price,
            "stock_position": self.stock_position,
            "total_value": self.total_value,
            "cash_value": self.cash,
            "stock_value": self.stock_value,
        }


def calculate_pnl(records: list[Record]) -> np.ndarray:
    total_values = np.array([record.total_value for record in records])
    initial_value = total_values[0]
    pnl = (total_values - initial_value) / initial_value
    return pnl


def calculate_max_drawdown(records: list[Record]) -> float:
    total_values = np.array([record.total_value for record in records])
    peak = np.maximum.accumulate(total_values)
    drawdown = (peak - total_values) / peak
    max_drawdown = np.max(drawdown)
    return max_drawdown


def to_dataframe(records: list[Record]):
    records_data = [record.to_dict() for record in records]
    return pd.DataFrame(records_data)
