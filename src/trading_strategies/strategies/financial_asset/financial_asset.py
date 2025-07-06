from abc import abstractmethod
from datetime import datetime

from src.trading_strategies.strategies.financial_asset.price import Price, EmptyPrice
from src.trading_strategies.strategies.financial_asset.symbol import Symbol


class FinancialAsset:
    def __init__(self, symbol: Symbol, price: Price):
        self._symbol = symbol
        self._price = price

    def get_price_numeric(self) -> float:
        return self._price.price()

    def get_price(self) -> Price:
        return self._price

    def get_t0(self) -> datetime:
        return self._price.time()

    def close_at_price(self, price_t: Price | float) -> [float]:
        # close asset and return the profits (loss)
        # only current price for stock
        # calculate payoff for options
        return [0]

    def commission_fee(self, optional=None) -> [float]:
        return 0

    def is_same_symbol(self, other: Symbol) -> bool:
        assert isinstance(other, Symbol)
        return self._symbol.__eq__(other)

    def symbol(self) -> Symbol:
        return self._symbol

    @abstractmethod
    def __str__(self):
        pass


class EmptyAsset(FinancialAsset):

    def __init__(self):
        symbol = Symbol("Empty")
        price = EmptyPrice()
        super().__init__(symbol, price)

    def __str__(self):
        return f"Empty asset created at {self._price.time()}"
