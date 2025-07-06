import math
import sys
from typing import List

from src.trading_strategies.strategies.financial_asset.symbol import Symbol
from src.trading_strategies.util.exception import ExceptionHandler


def daily_vol_to_annual(vol: float):
    return vol * math.sqrt(252)


def find_target(target: float, values: [float], larger=True):
    """

    :param target: The value to search from.
    :param values: Available values.
    :param larger: Choose the larger one to break ties.
    :return: The found closest value in a collection of `values`
    with respect to `target`.

    @author: Huanjie Zhang
    """
    if len(values) == 0:
        ExceptionHandler.raise_illegal_arguments("There is no value to search in.")
        return 0.0
    min_diff = sys.maxsize
    result = values[0]
    for value in values:
        delta = value - target
        if abs(delta) < min_diff:
            result = value
            min_diff = abs(delta)
        elif abs(delta) == min_diff:
            # break ties
            if value > result and larger:
                result = value
            elif value < result and not larger:
                result = value
    return result


def match_strike(target_strike: float, premiums: dict[float, float], larger=True) -> (float, float):
    """
    :param target_strike: The wanted strike price, not necessarily in the pool of strikes.
    :param premiums: A map of (strike price -> premium).
    :param larger: Break ties choosing the larger strike.
    :return: A tuple (strike, premium) found in `premiums`.

    @author: Huanjie Zhang
    """
    strike = find_target(target_strike, [x for x in premiums.keys()], larger)
    if strike == 0.0:
        return .0, .0
    return strike, premiums[strike]


def to_string(symbols: List[Symbol]):
    return [x.symbol for x in symbols]
