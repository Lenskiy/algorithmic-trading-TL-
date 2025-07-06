from datetime import datetime
from typing import List

from src.trading_strategies.data_access.data_access import DataAccess
from src.trading_strategies.data_access.risk_free_rate import RatePeriod
from src.market.order import Order, EmptyOrder
from src.trading_strategies.strategies.financial_asset.option import Option, PutOption, CallOption
from src.trading_strategies.strategies.financial_asset.price import Price
from src.trading_strategies.strategies.financial_asset.stock import Stock
from src.trading_strategies.strategies.financial_asset.symbol import Symbol
from src.trading_strategies.strategies.strategy.option_strategy.calculators.option_pricing import bsm_pricing2
from src.trading_strategies.strategies.strategy.option_strategy.calculators.option_strike import get_closest_strike
from src.trading_strategies.strategies.strategy.strategy_id import StrategyId


class MarketSingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class SimulatedMarket(metaclass=MarketSingletonMeta):
    def __init__(self):
        self._order_book = []

    def submit_order(self, orders: List[Order], agent, strategy_id: StrategyId):
        order_accepted = False
        stock_price = 0
        for order in orders:
            if isinstance(order, EmptyOrder):
                continue
            symbol = order.asset.symbol()
            date = order.date
            # add premium to option in order
            stock_price = DataAccess().get_stock_price_at(symbol, date)
            if isinstance(order.asset, Option):
                target_strike = order.asset.get_strike().price()
                expiry = order.asset.get_expiry()
                is_put = isinstance(order.asset, PutOption)
                strike_price, premium = self._simulate_option_price(
                    symbol, stock_price, target_strike, date, expiry, is_put)
                if premium <= 0 or strike_price <= 0:
                    # cannot simulate the premium due to not enough data in calculating vol.
                    order.reject_order()
                    return

                if is_put:
                    new_option = PutOption(symbol, Price(strike_price, date), expiry, Price(premium, date))
                else:
                    new_option = CallOption(symbol, Price(strike_price, date), expiry, Price(premium, date))
                order.asset = new_option
            order.accept_order()

    @staticmethod
    def _simulate_option_price(symbol: Symbol, stock_price: float, target_strike: float,
                               date: datetime, expiry: datetime, is_put=True):
        strike_price = get_closest_strike(stock_price, target_strike)
        rf = DataAccess().get_risk_free(date, RatePeriod.TEN_YEAR)
        premium = bsm_pricing2(Stock(symbol, Price(stock_price, date)), strike_price, expiry, [], rf.value, is_put)
        return strike_price, premium
