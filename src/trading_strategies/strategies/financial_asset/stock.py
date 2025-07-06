from datetime import timedelta

import numpy as np
import statistics

from src.trading_strategies.data_access.data_access import DataAccess
from src.trading_strategies.strategies.financial_asset.financial_asset import FinancialAsset
from src.trading_strategies.strategies.financial_asset.price import Price
from src.trading_strategies.strategies.financial_asset.symbol import Symbol

# file_path = '../src/data/sp500_adj_close_prices.csv'
file_path = 'data/sp500_adj_close_prices.csv'


class Stock(FinancialAsset):
    def __init__(self, symbol: Symbol, price: Price):
        super().__init__(symbol, price)
        # self.historical_price = historical_price
        # self.volatility = self.calculate_volatility()

    def calculate_volatility(self):
        returns = self.get_returns()
        volatility = statistics.stdev(returns)
        return volatility

    def calculate_expected_return(self):
        returns = self.get_returns()
        return sum(returns) / len(returns)

    def get_returns(self):
        prices = [price for price in self.get_prices()]
        prices = [p for p in prices if not np.isnan(p)]
        returns = np.diff(prices) / prices[:-1]
        return returns

    def get_prices(self):
        prev_date = self._price.time() - timedelta(days=182)
        data = DataAccess().get_stock([self.symbol()], prev_date, self._price.time())
        return data.iloc[:, 1]

    def update_price(self, price: Price):
        self._price = price

    def __str__(self):
        return f"Stock {self.symbol}: {self._price}"
