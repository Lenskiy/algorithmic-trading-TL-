from datetime import datetime

from src.trading_strategies.strategies.financial_asset.financial_asset import FinancialAsset
from src.trading_strategies.agent.transactions.position import Position
from src.trading_strategies.agent.transactions.positions import Positions


class Transaction:

    def __init__(self, positions: Positions, asset: FinancialAsset, asset_name: str, time: datetime, initial_margin: float, msg=""):
        self._positions = positions
        self._asset = asset
        self._asset_name = asset_name
        self._time = time
        self._msg = msg
        self._realised_payoff = .0
        self._initial_margin = initial_margin

    def append_msg(self, msg=""):
        self._msg += msg

    def get_asset(self):
        return self._asset

    def get_asset_name(self):
        return self._asset_name

    def get_time(self):
        return self._time

    def is_short(self) -> bool:
        return self._positions.position == Position.SHORT

    def is_long(self) -> bool:
        return self._positions.position == Position.LONG

    def realise_payoff(self, amount: float):
        self._realised_payoff += amount

    def get_payoff(self):
        return self._realised_payoff

    def __str__(self):
        s = (f"Transaction Entry:\n"
             f"  Positions: {self._positions.__str__()}\n"
             f"  Asset: {self._asset}\n"
             f"  Time: {self._time.strftime("%Y-%m-%d")}\n"
             f"  Initial margin: {self._initial_margin}")
        if self._msg != "":
            s += f"  Note: {self._msg}\n"
        return s
