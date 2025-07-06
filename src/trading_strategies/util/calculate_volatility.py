from datetime import datetime, timedelta

import numpy as np
from arch import arch_model

from src.trading_strategies.data_access.data_access import DataAccess
from src.trading_strategies.data_access.volatility import VolatilityType, Volatility, EmptyVolatility
from src.trading_strategies.strategies.financial_asset.symbol import Symbol


def calculate_vol(symbol: Symbol, volatility_type: VolatilityType, end: datetime, days=182) -> Volatility:
    start = end - timedelta(days=days)
    if volatility_type == VolatilityType.GARCH:
        try:
            value = calculate_garch(symbol, start, end)
        except ValueError:
            return EmptyVolatility()
    else:
        value = 0
    return Volatility(value, volatility_type)


def get_returns(symbol: Symbol, start_date: datetime, end_date: datetime):
    # TODO: timeframe might not in the data_access
    da = DataAccess()
    data = da.get_stock([symbol], start_date, end_date)
    data = data[symbol.symbol].tolist()
    prices = [p for p in data if not np.isnan(p)]
    return np.diff(prices) / prices[:-1]


def calculate_garch(symbol: Symbol, start_date, end_date):
    returns = get_returns(symbol, start_date, end_date)
    model = arch_model(returns, vol='GARCH', p=1, q=1, rescale=False)
    fit = model.fit(disp='off')
    vol = np.sqrt(fit.forecast(horizon=100).variance).mean(axis=1).iloc[-1]
    return vol * np.sqrt(252)
