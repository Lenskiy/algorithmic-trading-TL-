import datetime
from typing import Set, Dict, List, Tuple

from src.trading_strategies.agent.performance import calculate_option_profit, calculate_drawdowns
from src.trading_strategies.agent.transactions.transaction import Transaction
from src.trading_strategies.data_access.data_package import DataPackage
from src.trading_strategies.agent.margins import Margins
from src.trading_strategies.backtesting.simulated_market import SimulatedMarket
from src.trading_strategies.strategies.financial_asset.financial_asset import FinancialAsset, EmptyAsset
from src.trading_strategies.strategies.financial_asset.symbol import Symbol
from src.trading_strategies.agent.margin_calculator import get_margin_calculator
from src.trading_strategies.strategies.strategy.strategy import Strategy
from src.trading_strategies.strategies.strategy.strategy_id import StrategyId
from src.trading_strategies.agent.transactions.transactions import Transactions


class Agent:
    # used in agent-based back testing. Assume to be no agents in simple daily market replay (single agent).
    # -strategies: Map < StrategyId, Strategy >
    # -transactions: Map < StrategyId, Transactions >

    def __init__(self, strategies: dict[StrategyId, Strategy]):
        self._strategies = strategies
        self._all_transactions = dict[StrategyId, Transactions]()
        for strategy_id, strategy in strategies.items():
            self._all_transactions[strategy_id] = Transactions(strategy_id)
            strategy.register_agent(self)
        self._market = SimulatedMarket()
        self._assets: Dict[StrategyId, List[Tuple[FinancialAsset, str]]] = dict()
        self._margins: Dict[StrategyId, Margins] = dict()

    def trading_symbol(self, symbol: Symbol):
        for strategy_id, strategy in self._strategies.items():
            if not strategy.is_same_symbol(symbol):
                continue
            if strategy_id not in self._assets:
                continue
            if len(self._assets[strategy_id]) > 0:
                return True
        return False

    def get_asset(self, strategy_id: StrategyId) -> List[Tuple[FinancialAsset, str]]:
        if strategy_id not in self._assets.keys():
            return [(EmptyAsset(), "Empty")]
        return self._assets[strategy_id]

    def get_asset_by_name(self, strategy_id: StrategyId, target: str) -> FinancialAsset:
        if strategy_id not in self._assets.keys():
            return EmptyAsset()
        assets = self._assets[strategy_id]
        for asset, asset_name in assets:
            if asset_name == target:
                return asset
        return EmptyAsset()

    def _update_asset(self, strategy_id: StrategyId, asset: FinancialAsset, name: str):
        if strategy_id not in self._assets.keys():
            self._assets[strategy_id] = list()
        assets_list = self._assets[strategy_id]
        for i in range(len(assets_list)):
            if assets_list[i][1] == name:
                assets_list[i] = (asset, name)
                return
        assets_list.append((asset, name))

    def get_symbols(self) -> Set:
        result: Set[Symbol] = set()
        for strategy in self._strategies.values():
            result.add(strategy.symbol())
        return result

    def update(self, symbol: Symbol, new_data: DataPackage) -> bool:
        # return update is successful or not
        for strategy_id, strategy in self._strategies.items():
            if symbol != strategy.symbol():
                continue
            orders = strategy.update(new_data)
            self._market.submit_order(orders, agent=self, strategy_id=strategy_id)

            if any([not order.is_successful() for order in orders]):
                continue
            # order is successful
            stock_price = new_data.stock.get_price().price()
            date = new_data.date
            # update assets
            for order in orders:
                if order.is_successful():
                    self._update_asset(strategy_id, order.asset, order.asset_name)
            # update margin
            self._update_initial_margin(strategy_id, stock_price, date)
            margin = self._margins[strategy_id].peak_last()[1]
            for order in orders:
                if order.is_successful():
                    self._update_payoff(strategy_id, order.asset_name, stock_price)
                    transaction = Transaction(positions=order.positions,
                                              asset=order.asset,
                                              asset_name=order.asset_name,
                                              time=order.date,
                                              initial_margin=margin, msg=order.msg)
                    self._all_transactions[strategy_id].add_transaction(transaction)
        return True

    def need_update(self, date: datetime) -> bool:
        return any([strategy.need_update(date) for strategy in self._strategies.values()])

    def need_update_for(self, date: datetime, symbol: Symbol):
        for strategy in self._strategies.values():
            if strategy.is_same_symbol(symbol):
                if strategy.need_update(date):
                    return True
        return False

    def get_all_transactions(self) -> dict[StrategyId, Transactions]:
        return self._all_transactions

    def evaluate(self):
        dates = dict[StrategyId, list[float]]()
        payoffs = dict[StrategyId, list[float]]()
        profits = dict[StrategyId, list[float]]()
        cumulative_profits = dict[StrategyId, list[tuple[float, float]]]()
        drawdowns = dict[StrategyId, list[float]]()

        for strategy_id, transactions in self._all_transactions.items():
            (dates[strategy_id],
             payoffs[strategy_id],
             profits[strategy_id],
             cumulative_profits[strategy_id]) = calculate_option_profit(transactions)
            drawdowns[strategy_id] = calculate_drawdowns(cumulative_profits[strategy_id])
        return (dates,
                payoffs,
                profits,
                cumulative_profits,
                drawdowns)

    # ====== payoffs
    def cal_payoff(self, strategy_id: StrategyId, stock_price: float, num_t=1):
        for t in self._all_transactions[strategy_id].last_n(num_t):
            payoff = t.get_asset().option_payoff(stock_price)
            t.realise_payoff(payoff)
            t.append_msg(f"Realised payoff: {payoff}.\n")

    def _update_payoff(self, strategy_id: StrategyId, asset_name: str, stock_price: float):
        transactions = self._all_transactions[strategy_id].get_transactions()
        if len(transactions) == 0:
            return
        for i in range(len(transactions) - 1, -1, -1):
            if transactions[i].get_asset_name() == asset_name:
                payoff = transactions[i].get_asset().option_payoff(stock_price)
                transactions[i].realise_payoff(payoff)
                transactions[i].append_msg(f"Realised payoff: {payoff}.\n")
                return

    # ===== margins
    def _update_initial_margin(self, strategy_id: StrategyId, stock_price: float, date: datetime):
        margin = self._cal_margin(strategy_id, stock_price)
        if strategy_id not in self._margins.keys():
            self._margins[strategy_id] = Margins()
        self._margins[strategy_id].append_margin(date, margin, True)

    def _update_maintenance_margin(self, strategy_id: StrategyId, stock_price: float, date: datetime):
        margin = self._cal_margin(strategy_id, stock_price)
        self._margins[strategy_id].append_margin(date, margin, False)

    def notify_maintenance_margin(self, date: datetime, stock_price: float, symbol: Symbol):
        for strategy_id, strategy in self._strategies.items():
            if not strategy.is_same_symbol(symbol):
                continue
            if strategy_id not in self._margins.keys():
                self._margins[strategy_id] = Margins()
            last_margin_date, margin = self._margins[strategy_id].peak_last()
            if last_margin_date < date:
                self._update_maintenance_margin(strategy_id, stock_price, date)

    def _cal_margin(self, strategy_id: StrategyId, stock_price: float):
        strategy = self._strategies[strategy_id]
        symbol = strategy.symbol()
        assets = [x[0] for x in self._assets[strategy_id]]
        margin = get_margin_calculator(symbol).calculate_margin(strategy.margin_type, stock_price, assets)
        return margin

    def get_margins(self):
        return self._margins
