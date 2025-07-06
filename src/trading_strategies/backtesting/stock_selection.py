# 1 x stock with low or possibly negative beta (say beta < 0.2).
# 1 x stock which has paid a dividend every year for at least 10 years
import statistics

import numpy as np

from src.trading_strategies.util.read_file import read_file


class StockSelection:
    def __init__(self):
        low_vol = ["KO", "JNJ"]
        high_vol = ["SMCI", "ENPH"]
        high_market_cap = ["MSFT", "AAPL", "NVDA", "GOOG"]
        low_market_cap = ["CMA", "MHK"]
        # https://medium.com/@businessandbooks/10-smallest-companies-in-the-s-p500-index-df72498a788b
        negative_beta = []
        dividends_paying_10years = ["GIS"]
        self.full = low_vol + high_vol + low_market_cap + high_market_cap + negative_beta + dividends_paying_10years
        self.simple = ["AAPL"]


def dividend_analysis():
    dividend_file = "../data/sp500_dividends.csv"
    data = read_file(dividend_file)
    symbols = data.columns.tolist().remove("Date")


def main():
    stock_filename = "../data/sp500_adj_close_prices.csv"
    data = read_file(stock_filename)
    result = []
    for symbol in data.columns[1:]:
        try:
            tmp = data[symbol]
            prices = data[symbol].tail(min(500, len(tmp))).tolist()
            returns = np.diff(prices) / prices[:-1]
            result.append((round(statistics.stdev(returns), 4), symbol))
        except AttributeError:
            pass
    result = sorted(result)

    print(result[0], result[1], result[2])
    print(result[len(result) - 1], result[len(result) - 2], result[len(result) - 3])

    low_vol = ["KO", "JNJ"]
    high_vol = ["SMCI", "ENPH"]
    high_market_cap = ["MSFT", "AAPL", "NVDA", "GOOG"]
    low_market_cap = ["CMA",
                      "MHK"]  # https://medium.com/@businessandbooks/10-smallest-companies-in-the-s-p500-index-df72498a788b
    negative_beta = [
        "WDFC"]  # https://www.suredividend.com/negative-beta-stocks/#:~:text=A%20beta%20of%201.0%20means,precisely%20opposite%20the%20S%26P%20500
    dividends_paying_10years = ["GIS"]

    dividend_analysis()


# (0.0106, 'KO') (0.0107, 'JNJ') (0.0108, 'MCD')
# (0.0447, 'SMCI') (0.0423, 'ENPH') (0.0422, 'EPAM')


if __name__ == "__main__":
    main()
