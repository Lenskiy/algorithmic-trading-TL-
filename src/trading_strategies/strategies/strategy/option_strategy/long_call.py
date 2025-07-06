from datetime import datetime

from src.trading_strategies.strategies.financial_asset.option import Option
from src.trading_strategies.strategies.financial_asset.symbol import Symbol
from src.trading_strategies.strategies.strategy.option_strategy.option_strategy import OptionStrategy
from src.trading_strategies.strategies.strategy.option_strategy.calculators.option_strike import calculate_strike
from src.trading_strategies.strategies.strategy.strategy_id import StrategyId
from src.trading_strategies.agent.transactions.position import Position
from src.trading_strategies.util.expiry_date import next_expiry_date

risk_free_rate = 0.03


class LongCall(OptionStrategy):

    def __init__(self, strategy_id: StrategyId, symbol: Symbol, is_itm: bool,
                 is_weekly: bool, weekday, num_of_strikes, scale=1, max_strike=True, cross_over=True, same_expiration=True, parent=None):
        super().__init__(strategy_id, symbol, is_itm, is_weekly,
                         weekday, num_of_strikes, scale)
        self._position = Position.LONG
        self.asset_name = "long_call"

    def roll_over(self, stock_price: float, date: datetime, prev_option=None) -> (float, datetime):
        strike_price = calculate_strike(stock_price, self._is_itm, self._num_of_strikes, False)
        expiration_date = next_expiry_date(date, is_weekly=self._is_weekly, weekday=self._weekday)
        return strike_price, expiration_date

    def roll_down(self, stock_price: float, date: datetime, prev_option: Option) -> (float, datetime):
        return self.roll_over(stock_price, date, prev_option)

    def roll_up(self, stock_price: float, date: datetime, prev_option: Option) -> (float, datetime):
        return self.roll_over(stock_price, date, prev_option)
