from datetime import datetime

import numpy as np
import pandas as pd

from src.trading_strategies.agent.margins import Margins
from src.trading_strategies.strategies.strategy.strategy_id import StrategyId


class BacktestingSummary:
    def __init__(self, initial_cash_value,
                 end_cash_value,
                 dates: dict[StrategyId, list[datetime]],
                 profits: dict[StrategyId, list[float]],
                 payoffs: dict[StrategyId, list[float]],
                 cumulative_profits: dict[StrategyId, list[float]],
                 drawdowns: dict[StrategyId, list[float]],
                 margins: dict[StrategyId, Margins],
                 years=1):  # , profits: float[], drawndowns: float[], cagr: float):
        self._initial_cash_value = initial_cash_value
        self._end_cash_value = end_cash_value
        self._data = dict[StrategyId, pd.DataFrame]()
        self.margins = margins
        # convert to data frame
        for strategy_id in dates.keys():
            self._data[strategy_id] = pd.DataFrame({
                "Date": dates[strategy_id],
                "Payoffs": payoffs[strategy_id],
                "Profit": profits[strategy_id],
                "Cumulative": cumulative_profits[strategy_id],
                "Drawdown": drawdowns[strategy_id]
            })

        self._years = years
        if initial_cash_value == 0:
            self._cagr = 0
        else:
            self._cagr = (1 + end_cash_value / initial_cash_value) ** (1 / years) - 1

    def get_cagr(self):
        return self._cagr

    def get_data(self) -> dict[StrategyId, pd.DataFrame]:
        return self._data

    def max_drawdown(self, strategy_id: StrategyId):
        return np.max(self._data[strategy_id]["Drawdown"])

    def __str__(self):
        result = ""
        for strategy_id in self._data.keys():
            result += (f"{strategy_id} traded for {self._years} years: \n"
                       f"  {self._data[strategy_id].round(2)} \n"
                       f"=========================== \n\n")
        return result
