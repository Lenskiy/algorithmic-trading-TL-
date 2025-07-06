import numpy as np

from src.trading_strategies.strategies.financial_asset.stock import Stock


def simulate_portfolio(stock_list: [Stock], time_period, num_steps):
    num_assets = len(stock_list)
    time_steps = np.linspace(0., time_period, int(num_steps))
    A = np.linalg.cholesky(covariance_matrix(stock_list))
    portfolio_values = np.zeros([num_assets, int(num_steps)])
    portfolio_values[:, 0] = np.array([stock.get_price() for stock in stock_list])

    for stock in stock_list:
        for i in range(1, int(num_steps)):
            drift = (stock.calculate_expected_return() - 0.5 * stock.volatility ** 2) * (time_steps[i] - time_steps[i - 1])
            Z = np.random.normal(0., 1., num_assets)
            diffusion = np.matmul(A, Z) * (np.sqrt(time_steps[i] - time_steps[i - 1]))
            portfolio_values[:, i] = portfolio_values[:, i - 1] * np.exp(drift + diffusion)

    return portfolio_values

def covariance_matrix(stock_list: [Stock]):
    num_stocks = len(stock_list)
    historical_prices = [stock.historical_price for stock in stock_list]
    num_data_points = min(len(prices) for prices in historical_prices)
    returns_matrix = np.zeros((num_stocks, num_data_points))

    for i, prices in enumerate(historical_prices):
        returns = np.diff([price.price for price in prices]) / [price.price for price in prices[:-1]]
        returns_matrix[i] = returns[:num_data_points]

    cov_matrix = np.cov(returns_matrix)

    return cov_matrix

# def calculate_var(stock: Stock, option: Option, risk_free_rate, confidence_level, num_paths, num_steps):
#     time_to_maturity = (option.expiration_date - stock.current_price.timeStamp) / datetime.timedelta(days=365)
#     stock_price_paths = simulate_stock_price(stock, risk_free_rate, time_to_maturity, num_paths, num_steps)
#     option_payoff = option.option_payoff(stock_price_paths[:, -1], option.strike_price.price)
#
#     # Calculate option P&L
#     option_pl = option_payoff - option.premium
#     option_pl_sorted = np.sort(option_pl)
#
#     # Calculate VaR
#     var_index = int(num_paths * (1 - confidence_level))
#     option_var = -option_pl_sorted[var_index]
#
#     return option_var

def stress_testing():
    # Implement stress testing analysis
    pass

def sensitivity_analysis():
    # Implement sensitivity analysis
    pass