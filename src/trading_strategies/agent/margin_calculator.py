from abc import abstractmethod
from enum import Enum
from typing import Callable, List

from src.trading_strategies.strategies.financial_asset.option import Option, CallOption, PutOption, EmptyOption
from src.trading_strategies.strategies.financial_asset.symbol import Symbol


class MarginType(Enum):
    SHORT_CALL = "short_call"
    SHORT_PUT = "short_put"
    STRADDLE = "straddle"
    SPREAD = "diagonal"
    NOT_REQUIRED = "not_required"


class MarginCalculator:

    @abstractmethod
    def __init__(self, margin_para1: float, margin_para2: float):
        self.margin_para1 = margin_para1
        self.margin_para2 = margin_para2

    def calculate_margin(self, margin_type: MarginType, stock_price: float, options: [Option]):
        if margin_type == MarginType.SPREAD:
            margin = self.spread_margin(options[0].get_strike().price(), options[1].get_strike().price())
            return round(margin, 2)
        if margin_type == MarginType.STRADDLE:
            call = EmptyOption()
            put = EmptyOption()
            for option in options:
                if isinstance(option, CallOption):
                    call = option
                elif isinstance(option, PutOption):
                    put = option
            margin = self.straddle_margin(stock_price, call.get_strike().price(), call.get_price().price(),
                                          put.get_strike().price(), put.get_price().price())
            return round(margin, 2)
        calculator = self._choose_calculator(margin_type)
        margin = self._calculator_proxy(calculator, margin_type, stock_price, options)
        return round(margin, 2)

    @staticmethod
    def _calculator_proxy(calculator: Callable, margin_type: MarginType, stock_price: float, options: List[Option]):
        if len(options) == 1:
            option = options[0]
            return calculator(stock_price, option.get_strike().price(), option.get_price().price())

    @staticmethod
    def _infer_margin_type(options: List[Option]):
        if len(options) == 2:
            if type(options[0]) != type(options[1]):
                return MarginType.STRADDLE
            else:
                return MarginType.SPREAD
        option = options[0]
        if isinstance(option, CallOption):
            return MarginType.SHORT_CALL
        if isinstance(option, PutOption):
            return MarginType.SHORT_PUT
        return MarginType.NOT_REQUIRED

    def _choose_calculator(self, margin_type: MarginType) -> Callable:
        if margin_type == MarginType.SHORT_CALL:
            return self.naked_call_margin
        if margin_type == MarginType.SHORT_PUT:
            return self.naked_put_margin
        if margin_type == MarginType.STRADDLE:
            return self.straddle_margin
        if margin_type == MarginType.SPREAD:
            return self.spread_margin
        return self.zero_margin

    @staticmethod
    def zero_margin(*args):
        return 0

    def _margin1(self, underlying_value: float, otm_amount: float, premium: float):
        return self.margin_para1 * underlying_value - otm_amount + premium

    def _margin2(self, contract_value: float, premium: float):
        return self.margin_para2 * (contract_value + premium)

    def naked_call_margin(self, underlying_value, strike_price, premium):
        otm_amount = max(0, strike_price - underlying_value)
        margin1 = self._margin1(underlying_value, otm_amount, premium)
        margin2 = self._margin2(underlying_value, premium)
        margin3 = 1
        return max(margin1, margin2, margin3)

    def naked_put_margin(self, underlying_value: float, strike_price: float, premium: float):
        otm_amount = max(.0, underlying_value - strike_price)
        margin1 = self._margin1(underlying_value, otm_amount, premium)
        margin2 = self._margin2(strike_price, premium)
        margin3 = 1
        return max(margin1, margin2, margin3)

    def straddle_margin(self, underlying_value: float, call_strike_price: float, call_premium: float,
                        put_strike_price: float, put_premium: float):
        naked_call_margin = self.naked_call_margin(underlying_value, call_strike_price, call_premium)
        naked_put_margin = self.naked_put_margin(underlying_value, put_strike_price, put_premium)
        if naked_call_margin > naked_put_margin:
            return naked_call_margin + put_premium
        else:
            return naked_put_margin + call_premium

    def spread_margin(self, strike_short, strike_long):
        return abs(strike_long - strike_short)


class EquityMarginCalculator(MarginCalculator):
    def __init__(self):
        super().__init__(margin_para1=0.2, margin_para2=0.1)


class IndexMarginCalculator(MarginCalculator):
    def __init__(self):
        super().__init__(margin_para1=0.15, margin_para2=0.1)


def get_margin_calculator(symbol: Symbol):
    return EquityMarginCalculator()
