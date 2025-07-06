import json
import os
from datetime import datetime, timedelta
from typing import Dict, Tuple

import numpy as np
import pandas as pd

from src.trading_strategies.data_access.data_access_meta import DataSingletonMeta
from src.trading_strategies.data_access.risk_free_rate import RatePeriod, RiskFree
from src.trading_strategies.data_access.volatility import Volatility, VolatilityType, EmptyVolatility

from src.trading_strategies.strategies.financial_asset.symbol import Symbol

stock_filename = "data/sp500_adj_close_prices.csv"
stock_date_format = "%Y-%m-%d %H:%M:%S"  # 2004-01-02 00:00:00
stock_date_column_name = "Date"
tbills_filename = "data/T-Bills.csv"
tbills_date_format = "%d/%m/%Y"  # 16/01/2004
tbills_date_column_name = "DATE"


class DataAccess(metaclass=DataSingletonMeta):
    def __init__(self):
        self._historical_stock: pd.DataFrame = pd.DataFrame()
        self._stock_price_file = {"filename": "data/sp500_adj_close_prices.csv",
                                  "date_format": "%Y-%m-%d %H:%M:%S",  # 2004-01-02 00:00:00
                                  "date_column_name": "Date"
                                  }
        self._t_bills_file = {"filename": "data/T-Bills.csv",
                              "date_format": "%d/%m/%Y",  # 02-01-2004
                              "date_column_name": "DATE"
                              }

        self._volatilities: Dict[Tuple[Symbol, VolatilityType, datetime], Volatility] = dict()
        self._risk_free: Dict[Tuple[RatePeriod, datetime], RiskFree] = dict()

    def get_volaitlity(self, symbol: Symbol, volatility_type: VolatilityType, date: datetime):
        entry = tuple((symbol, volatility_type, date))
        if entry not in self._volatilities.keys():
            from src.trading_strategies.util.calculate_volatility import calculate_vol
            tmp = calculate_vol(symbol, volatility_type, date)
            if isinstance(tmp, EmptyVolatility):
                return tmp
            self._volatilities[entry] = calculate_vol(symbol, volatility_type, date)
        return self._volatilities[entry]

    # ==== risk free
    def get_risk_free(self, date: datetime, rate_period=RatePeriod.TEN_YEAR):
        return RiskFree(0.03, rate_period, date)
        # entry = tuple((rate_period, date))
        # if entry not in self._risk_free.keys():
        #     self._risk_free[entry] = RiskFree(0, rate_period, date)
        # return self._risk_free[entry]

    # def retrieve_rf(date: datetime):
    #     result = _retrieve_by_date(tbills_filename, tbills_date_column_name, date, tbills_date_format)
    #     traceback_days = 10
    #     i = 0
    #     # in case risk free is not available on that date, search back a few days before.
    #     while len(result) == 0 and i < traceback_days:
    #         date = date - timedelta(days=1)
    #         result = _retrieve_by_date(tbills_filename, tbills_date_column_name, date, tbills_date_format)
    #         i += 1
    #     if len(result) == 0:
    #         return DataAccessResult(None)
    #     result = result["DTB3"].values[0]
    #     if result == ".":
    #         return retrieve_rf(date - timedelta(days=1))
    #     return DataAccessResult(float(result) / 100, True)

    # ==== stock
    def is_trading_in_historical(self, date: datetime) -> bool:
        dates = self._historical_stock[self._stock_price_file["date_column_name"]]
        date = date.strftime(self._stock_price_file["date_format"])
        return date in set(dates)
        # FIXME: may result in high computation time

    def _add_stock(self, stock_df: pd.DataFrame):
        self._historical_stock.add(stock_df)

    def refresh(self):
        self._historical_stock = pd.DataFrame()
        self._risk_free = pd.DataFrame()

    def get_stock_price_at(self, symbol: Symbol, date: datetime) -> float:
        # use case: has already store all necessary data
        tmp = self.get_stock([symbol], date, date)
        if len(tmp) > 0:
            return tmp[symbol.symbol].iloc[0]
        return np.nan

    def get_stock(self, symbols: [Symbol], start_date: datetime, end_date: datetime = None, refresh=False):
        symbols = [x.symbol for x in symbols]
        if refresh:
            self.refresh()
        missing_symbols = [x for x in symbols if not self.has_stock_data(x)]
        if len(missing_symbols) > 0:
            self._request_stock_from_local(missing_symbols, start_date, end_date)
        columns = [self._stock_price_file["date_column_name"]] + symbols
        dates = self._historical_stock[self._stock_price_file["date_column_name"]]
        s = start_date.strftime(self._stock_price_file["date_format"])
        e = end_date.strftime(self._stock_price_file["date_format"])
        rows = (s <= dates) & (dates <= e)
        return self._historical_stock[rows][columns]

    def has_stock_data(self, symbol):
        if isinstance(symbol, Symbol):
            symbol = symbol.symbol
        return symbol in self._historical_stock

    def _request_stock_from_local(self, symbols, start_date: datetime, end_date: datetime = None):
        if len(symbols) == 0:
            return None
        if end_date is None or end_date < start_date:
            end_date = start_date

        date_format = self._stock_price_file["date_format"]
        col_name = self._stock_price_file["date_column_name"]

        data = _read_csv(self._stock_price_file["filename"])
        date_series = data[col_name]

        start_date = self._offset_date(start_date, date_format, date_series)
        end_date = self._offset_date(end_date, date_format, date_series)
        s = start_date.strftime(date_format)
        e = end_date.strftime(date_format)

        columns = [col_name]
        for symbol in symbols:
            if isinstance(symbol, Symbol):
                columns.append(symbol.symbol)
            else:
                columns.append(symbol)

        data = data[(data[col_name] >= s) & (data[col_name] <= e)][columns].map(
            lambda x: round(x, 2) if isinstance(x, (float, int)) else x)
        if len(self._historical_stock) == 0:
            self._historical_stock = data
        else:
            self._historical_stock = self._historical_stock.merge(data)

    @staticmethod
    def _offset_date(date: datetime, str_format, date_column):
        offset = 1
        # backward search
        while offset < 10:
            if not any(date_column == date.strftime(str_format)):
                date = date - timedelta(days=offset)
            else:
                return date

        # forward search
        date + timedelta(days=10)
        offset = 1
        while offset < 10:
            if not any(date_column == date.strftime(str_format)):
                date = date + timedelta(days=offset)
            else:
                return date


# def retrieve_stock(symbol: Symbol, date: datetime) -> DataAccessResult:
#     data = _retrieve_by_date(stock_filename, stock_date_column_name, date, stock_date_format)
#     if len(data) == 0 or symbol.symbol not in data.columns:
#         return DataAccessResult(None)
#     price = data[symbol.symbol].values[0]
#     if np.isnan(price):
#         return DataAccessResult(None)
#     stock = Stock(symbol, Price(price, date))
#     return DataAccessResult(stock, True)


# def retrieve_rf(date: datetime):
#     result = _retrieve_by_date(tbills_filename, tbills_date_column_name, date, tbills_date_format)
#     traceback_days = 10
#     i = 0
#     # in case risk free is not available on that date, search back a few days before.
#     while len(result) == 0 and i < traceback_days:
#         date = date - timedelta(days=1)
#         result = _retrieve_by_date(tbills_filename, tbills_date_column_name, date, tbills_date_format)
#         i += 1
#     if len(result) == 0:
#         return DataAccessResult(None)
#     result = result["DTB3"].values[0]
#     if result == ".":
#         return retrieve_rf(date - timedelta(days=1))
#     return DataAccessResult(float(result) / 100, True)
#
#
# def _retrieve_by_date(filename: str, col_name: str, date: datetime, date_format=""):
#     data = _read_file(filename)
#     date_str = date.strftime(date_format)
#     result = data[data[col_name] == date_str]
#     return data[data[col_name] == date_str]


def _read_file(file_path: str):
    if _get_file_type(file_path).lower() == 'csv':
        return _read_csv(file_path)
    elif _get_file_type(file_path).lower() == 'json':
        return _read_json(file_path)
    else:
        raise ValueError("Unsupported file format")


def _read_csv(file_path: str):
    return pd.read_csv(file_path)


def _read_json(file_path: str):
    with open(file_path, 'r') as f:
        content = json.load(f)
    return json.dumps(content)


def _get_file_type(file_path: str):
    _, file_extension = os.path.splitext(file_path)
    return file_extension[1:]
