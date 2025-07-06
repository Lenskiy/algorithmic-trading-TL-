import sys

from src.trading_strategies.strategies.financial_asset.price import Price


class StrikeSpec:
    """
    Determine the next strike price given a list of available strike prices.
    Find the closest one in market if user set a target.
    Try to find by #of strike_price_width away from stock price, if automated.

    @author: Huanjie Zhang
    """

    def __init__(self, strike_width: int, bull_widths, bear_widths, is_manual=False):
        self.strike_width = strike_width
        self.bull_widths = bull_widths
        self.bear_widths = bear_widths
        self.__is_manual = is_manual

    def get_strike(self, price: Price, strikes: [], is_bull=True):
        if self.__is_manual:
            return self.__get_nearest_strike(price, strikes)
        if is_bull:
            return self.__get_strike_bull(price, strikes)
        return self.__get_strike_bear(price, strikes)

    @staticmethod
    def __get_nearest_strike(target_strike, strikes: [], looking_up=True):
        if len(strikes) == 0:
            return target_strike
        min_diff = sys.maxsize
        strike = target_strike
        for price in strikes:
            diff = price - target_strike
            if not looking_up:
                diff *= -1
            if diff < min_diff:
                strike = price
                min_diff = diff
        return strike

    def __get_strike_bull(self, stock: Price, strikes):
        target = stock.price + self.strike_width * self.bull_widths
        return self.__get_nearest_strike(target, strikes)

    def __get_strike_bear(self, stock: Price, strikes):
        target = stock.price - self.strike_width * self.bear_widths
        target = max(0, target)
        return self.__get_nearest_strike(target, strikes)
