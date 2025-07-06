from src.trading_strategies.agent.transactions.transactions import Transactions


def calculate_option_payoff(transactions: Transactions) -> tuple[list, list]:
    payoffs = []
    cumulative_payoffs = []
    c = 0
    for transaction in transactions.get_transactions():
        p = transaction.get_payoff()
        if transaction.is_short():
            p *= -1
        payoffs.append(p)
        c += p
        cumulative_payoffs.append(p)
    return payoffs, cumulative_payoffs


def calculate_option_profit(transactions: Transactions):
    # at the time of next transaction, previous option is closed
    dates = []
    profits = []
    cumulative_profits = []
    payoffs = []
    c_profit = 0
    prev_transaction = None
    for transaction in transactions.get_transactions():
        cost = - transaction.get_asset().get_price_numeric()
        if transaction.is_short():
            cost *= -1
        payoff = 0
        if prev_transaction is not None:
            payoff = prev_transaction.get_payoff()
            if prev_transaction.is_short():
                payoff *= -1
        # profits
        profits.append(payoff)
        profits.append(cost)
        # payoffs
        payoffs.append(payoff)
        payoffs.append(0)

        c_profit += payoff
        cumulative_profits.append(c_profit)
        c_profit += cost
        cumulative_profits.append(c_profit)
        dates.append(transaction.get_time())
        dates.append(transaction.get_time())

        prev_transaction = transaction
    return dates, payoffs, profits, cumulative_profits


def calculate_drawdowns(values: [float]) -> (float, [float]):
    percentages = []
    peak = 0
    for value in values:
        peak = max(peak, value)
        if peak == 0:
            percentages.append(0)
        else:
            percentages.append((peak - value) / peak)
    return percentages

