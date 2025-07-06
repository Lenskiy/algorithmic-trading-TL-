import pandas as pd
import riskfolio as rp
from pypfopt.efficient_frontier import EfficientFrontier
from pypfopt import risk_models
from pypfopt import expected_returns


class PortfolioWeight:
    def __init__(self, method):
        self.method = method

    """
    returns are pandas dataframe with each stock as a column and each row as date
    """
    def calculate_weights(self, returns: pd.DataFrame, risk_free_rate):
        if self.method == 'markowitz':
            weights = self.markowitz_weights(returns)
        elif self.method == 'hierarchical risk parity':
            weights = self.hrp_weights(returns, risk_free_rate)
        else:
            raise ValueError("Unsupported method: {}".format(self.method))
        return weights

    def markowitz_weights(self, returns):
        expected_return = expected_returns.mean_historical_return(returns)
        covariance_matrix = risk_models.sample_cov(returns)
        ef = EfficientFrontier(expected_return, covariance_matrix)
        weights = ef.max_sharpe()  # Optimise weights for Sharpe ratio maximization
        return weights

    def hrp_weights(self, returns, risk_free_rate):
        port = rp.HCPortfolio(returns)
        weights = port.optimization(model='HRP', codependence='pearson', rm='MV',
                                    rf=risk_free_rate, linkage='single', leaf_order=True)
        return weights

    # TODO: format of output from markowitz and hrp are different (need further adjustment)
