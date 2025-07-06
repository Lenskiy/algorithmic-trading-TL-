import unittest
import datetime
from src.trading_strategies.strategies.financial_asset.price import Price
from src.trading_strategies.strategies.financial_asset.symbol import StockSymbol
from src.trading_strategies.strategies.financial_asset.stock import Stock


class TestVolatility(unittest.TestCase):
    def setUp(self):
        self.symbol = StockSymbol("AAPL")
        self.current_time = datetime.datetime.now()
        self.current_price = Price(100.0, self.current_time)
        self.historical_prices = [
            Price(90.0, self.current_time - datetime.timedelta(days=3)),
            Price(95.0, self.current_time - datetime.timedelta(days=2)),
            Price(105.0, self.current_time - datetime.timedelta(days=1))
        ]
        self.stock = Stock(self.symbol, self.current_price, self.historical_prices)

    def volatility_calculation(self):
        self.assertAlmostEqual(self.stock.calculate_volatility(), 0.0351, places=4)

    def volatility_calculation_empty_prices(self):
        empty_stock = Stock(self.symbol, self.current_price, [])
        self.assertEqual(empty_stock.calculate_volatility(), 0.0)


if __name__ == '__main__':
    unittest.main()
