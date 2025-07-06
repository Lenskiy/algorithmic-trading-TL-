from datetime import timedelta, datetime

import numpy as np

from src.trading_strategies.agent.agent import Agent
from src.trading_strategies.backtesting.backtester import Backtester
from src.trading_strategies.data_access.data_access import DataAccess
from src.trading_strategies.data_access.data_package import DataPackage
from src.trading_strategies.strategies.financial_asset.price import Price
from src.trading_strategies.strategies.financial_asset.stock import Stock
from src.trading_strategies.strategies.financial_asset.symbol import Symbol


class DailyMarketReplay(Backtester):
    def __init__(self, start_date: datetime, end_date: datetime, self_agent: Agent, agents: list[Agent]):
        super().__init__(start_date, end_date, self_agent, agents)

    def run_back_testing(self):
        self._has_tested = True
        date = self._start_date
        while date <= self._end_date:
            if DataAccess().is_trading_in_historical(date):
                self._update(self._self_agent, date)
                for agent in self._agents:
                    self._update(agent, date)
            date += timedelta(days=1)

    def _update(self, agent: Agent, date: datetime):
        for symbol in agent.get_symbols():
            stock_price = DataAccess().get_stock_price_at(symbol, date)
            if np.isnan(stock_price):
                continue
            self._update_strategy(agent, date, stock_price, symbol)
            if agent.trading_symbol(symbol):
                agent.notify_maintenance_margin(date, stock_price, symbol)



    @staticmethod
    def _update_strategy(agent: Agent, date: datetime, stock_price: float, symbol: Symbol):
        if not agent.need_update_for(date, symbol):
            return False
        data = DataPackage(date, Stock(symbol, Price(stock_price, date)))
        return agent.update(symbol, data)

