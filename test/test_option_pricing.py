import unittest
import datetime
from src.trading_strategies.strategies.financial_asset.option import CallOption, PutOption
from src.trading_strategies.strategies.financial_asset.stock import Stock
from src.trading_strategies.strategies.financial_asset.price import Price
from src.trading_strategies.strategies.financial_asset.symbol import Symbol
from src.trading_strategies.strategies.strategy.option_strategy.calculators.option_pricing import bsm_pricing


class TestOptionPricing(unittest.TestCase):
    def setUp(self):
        self.symbol = Symbol("AAPL")
        self.current_time = datetime.datetime.now()
        self.current_price = Price(100.0, self.current_time)
        self.historical_prices = [
            Price(90.0, self.current_time - datetime.timedelta(days=3)),
            Price(95.0, self.current_time - datetime.timedelta(days=2)),
            Price(105.0, self.current_time - datetime.timedelta(days=1))
        ]
        self.stock = Stock(self.symbol, self.current_price, self.historical_prices)

        self.expiration_date = self.current_time + datetime.timedelta(days=180)
        self.call_option = CallOption(self.symbol, Price(110.0, self.current_time), self.expiration_date, 0)
        self.put_option = PutOption(self.symbol, Price(110.0, self.current_time), self.expiration_date, 0)
        self.risk_free_rate = 0.05

    def test_call_option_pricing(self):
        call_price = bsm_pricing(self.stock, self.call_option, self.risk_free_rate)
        self.assertAlmostEqual(call_price, 12.6981, delta=0.0001)

    def test_put_option_pricing(self):
        put_price = bsm_pricing(self.stock, self.put_option, self.risk_free_rate)
        self.assertAlmostEqual(put_price, 20.0189, delta=0.0001)


if __name__ == '__main__':
    unittest.main()
