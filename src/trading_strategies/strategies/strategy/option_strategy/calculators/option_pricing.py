from datetime import timedelta, datetime

import numpy as np
from scipy.stats import norm
import math
from scipy.optimize import minimize

from src.trading_strategies.data_access.data_access import DataAccess
from src.trading_strategies.data_access.volatility import VolatilityType, EmptyVolatility
from src.trading_strategies.strategies.financial_asset.price import Price
from src.trading_strategies.strategies.financial_asset.stock import Stock
from src.trading_strategies.util.expiry_date import next_nth_trading_day

"""
@:param volatility: annual
@:param risk free rate: continuous
"""


def bsm_pricing2(stock: Stock, strike: float, expiry: datetime,
                 dividends: list[Price], risk_free_rate, is_put: bool):
    t0 = stock.get_t0()
    time_to_maturity = (expiry - t0) / timedelta(days=365)
    volatility = DataAccess().get_volaitlity(stock.symbol(), VolatilityType.GARCH, t0)
    if isinstance(volatility, EmptyVolatility):
        return -1
    else:
        volatility = volatility.value
    adjusted_price = adjust_dividends(stock, dividends, risk_free_rate)
    if is_put:
        return round(calculate_put_price(adjusted_price, strike, volatility, time_to_maturity, risk_free_rate), 2)
    else:
        return round(calculate_call_price(adjusted_price, strike, volatility, time_to_maturity, risk_free_rate), 2)


def bsm_pricing(stock: Stock, strike: float, expiration_date, dividends: list[Price], risk_free_rate, is_put: bool):
    time_to_maturity = (expiration_date - stock.get_price().time()) / timedelta(days=365)
    volatility = stock.garch()
    adjusted_price = adjust_dividends(stock, dividends, risk_free_rate)
    if is_put:
        return calculate_put_price(adjusted_price, strike, volatility, time_to_maturity, risk_free_rate)
    else:
        return calculate_call_price(adjusted_price, strike, volatility, time_to_maturity, risk_free_rate)


def calculate_call_price(stock_price, strike_price, volatility: float, time_to_maturity, risk_free_rate):
    d1 = calculate_d1(stock_price, volatility, strike_price, time_to_maturity, risk_free_rate)
    d2 = calculate_d2(d1, volatility, time_to_maturity)
    call_price = stock_price * norm.cdf(d1) - strike_price * math.exp(-risk_free_rate * time_to_maturity) * norm.cdf(d2)
    return call_price


def calculate_put_price(stock_price, strike_price, volatility: float, time_to_maturity, risk_free_rate):
    d1 = calculate_d1(stock_price, volatility, strike_price, time_to_maturity, risk_free_rate)
    d2 = calculate_d2(d1, volatility, time_to_maturity)
    put_price = strike_price * math.exp(-risk_free_rate * time_to_maturity) * norm.cdf(-d2) - stock_price * norm.cdf(
        -d1)
    return put_price


def calculate_d1(stock_price, volatility: float, strike_price, time_to_maturity, risk_free_rate):
    return (math.log(stock_price / strike_price) + (
            risk_free_rate + 0.5 * volatility ** 2) * time_to_maturity) / (
            volatility * np.sqrt(time_to_maturity))


def calculate_d2(d1, volatility: float, time_to_maturity):
    return d1 - volatility * np.sqrt(time_to_maturity)


def adjust_dividends(stock: Stock, dividends: [float], risk_free: float):
    stock_price = stock.get_price().price()
    for dividend in dividends:
        stock_price -= dividend.price() * math.exp(
            (stock.get_price().time() - dividend.time()) / timedelta(days=365) * risk_free)

    return stock_price


def implied_date(stock_price: float, date: datetime, strike_price: float, risk_free_rate: float, premium: float, volatility: float,
                 is_put: bool):
    result = minimize(
        lambda t: error_function(t, stock_price, strike_price, premium, volatility, risk_free_rate, is_put),
        x0=0.1,
        bounds=[(0.02, 2)],
        method='Nelder-Mead')
    implied_t = result.x[0]
    return next_nth_trading_day(date, int(implied_t * 252))


def error_function(t, s0: float, strike: float, premium: float, volatility: float, risk_free: float,
                   is_put: bool) -> bool:
    if is_put:
        return abs(calculate_put_price(s0, strike, volatility, t, risk_free) - premium)
    return abs(calculate_call_price(s0, strike, volatility, t, risk_free) - premium)
