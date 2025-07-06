from src.trading_strategies.strategies.strategy.strategy_id import StrategyId
from src.trading_strategies.agent.transactions.transaction import Transaction


class Transactions:

    def __init__(self, strategy_id: StrategyId):
        self._id = strategy_id
        self._transactions_list = list[Transaction]()

    def last_n(self, n: int):
        if n <= 0:
            return []
        l = len(self._transactions_list)
        if l < n:
            return []
        return self._transactions_list[(l - n): l]

    def get_id(self):
        return self._id

    def same_id(self, strategy_id: StrategyId):
        return self._id == strategy_id

    def add_transaction(self, transaction: Transaction):
        self._transactions_list.append(transaction)

    def get_transactions(self):
        return self._transactions_list

    def peak_last(self):
        if len(self._transactions_list) == 0:
            return None
        return self._transactions_list[len(self._transactions_list) - 1]

    def __str__(self):
        s = ""
        for t in self._transactions_list:
            s += t.__str__()
            s += "\n"
        return s
