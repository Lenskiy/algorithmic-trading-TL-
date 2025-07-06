from datetime import datetime

from src.trading_strategies.strategies.financial_asset.option import Option
from src.trading_strategies.strategies.financial_asset.stock import Stock


class DataPackage:

    def __init__(self, date: datetime, stock: Stock = None, option: Option = None):
        self.date = date
        self.stock = stock
        self.option = option

    def set_stock(self, stock: Stock):
        self.stock = stock

    def set_option(self, option: Option):
        self.option = option





