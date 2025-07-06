import unittest
from unittest.mock import MagicMock
from backtesting.market import Market
from backtesting.record import Record
from system_interface.msg import Order, TickData


class TestMarket(unittest.TestCase):
    def setUp(self):
        # Initialize the Market with 1000 initial cash
        self.market = Market(initial_cash=1000.0)

    def test_initialization(self):
        self.assertEqual(self.market.current_cash, 1000.0)
        self.assertEqual(self.market.current_position, 0.0)
        self.assertEqual(self.market.current_price, 0.0)
        self.assertEqual(self.market.current_time, 0)
        self.assertEqual(len(self.market.pending_orders), 0)
        self.assertEqual(len(self.market.records), 0)

    def test_update_market_data(self):
        # Create a mock TickData message
        tick_data = MagicMock(spec=TickData)
        tick_data.price = 105.0
        tick_data.time = 1234567890

        self.market.update_market_data(tick_data)

        self.assertEqual(self.market.current_price, 105.0)
        self.assertEqual(self.market.current_time, 1234567890)

    def test_submit_order(self):
        # Create a mock Order message
        order = MagicMock(spec=Order)
        order.side = "buy"
        order.price = 100.0
        order.size = 5.0
        order.time = 1234567890

        self.market.submit_order(order)

        self.assertEqual(len(self.market.pending_orders), 1)
        self.assertIn(order, self.market.pending_orders)

    def test_execute_order_buy_success(self):
        # Set current market price
        self.market.current_price = 100.0

        # Create a buy order that should be executed
        order = MagicMock(spec=Order)
        order.side = "buy"
        order.price = 100.0  # >= current_price
        order.size = 5.0
        order.time = 1234567890

        result = self.market.execute_order(order)

        self.assertTrue(result)
        self.assertEqual(self.market.current_position, 5.0)
        self.assertEqual(self.market.current_cash, 500.0)  # 1000 - 5*100

    def test_execute_order_buy_failure(self):
        # Set current market price
        self.market.current_price = 100.0

        # Create a buy order that should not be executed
        order = MagicMock(spec=Order)
        order.side = "buy"
        order.price = 99.0  # < current_price
        order.size = 5.0
        order.time = 1234567890

        result = self.market.execute_order(order)

        self.assertFalse(result)
        self.assertEqual(self.market.current_position, 0.0)
        self.assertEqual(self.market.current_cash, 1000.0)

    def test_execute_order_sell_success(self):
        # Pre-position for selling
        self.market.current_position = 5.0
        self.market.current_cash = 500.0
        self.market.current_price = 100.0

        # Create a sell order that should be executed
        order = MagicMock(spec=Order)
        order.side = "sell"
        order.price = 100.0  # <= current_price
        order.size = 2.0
        order.time = 1234567890

        result = self.market.execute_order(order)

        self.assertTrue(result)
        self.assertEqual(self.market.current_position, 3.0)
        self.assertEqual(self.market.current_cash, 700.0)  # 500 + 2*100

    def test_execute_order_sell_failure(self):
        # Pre-position for selling
        self.market.current_position = 5.0
        self.market.current_cash = 500.0
        self.market.current_price = 100.0

        # Create a sell order that should not be executed
        order = MagicMock(spec=Order)
        order.side = "sell"
        order.price = 101.0  # > current_price
        order.size = 2.0
        order.time = 1234567890

        result = self.market.execute_order(order)

        self.assertFalse(result)
        self.assertEqual(self.market.current_position, 5.0)
        self.assertEqual(self.market.current_cash, 500.0)

    def test_handle_pending_orders(self):
        # Set current market price and time
        self.market.current_price = 100.0
        self.market.current_time = 1234567890

        # Create two orders: one should execute, one should not
        order1 = MagicMock(spec=Order)
        order1.side = "buy"
        order1.price = 100.0  # >= current_price
        order1.size = 5.0
        order1.time = 1234567890

        order2 = MagicMock(spec=Order)
        order2.side = "buy"
        order2.price = 99.0  # < current_price
        order2.size = 3.0
        order2.time = 1234567890

        self.market.submit_order(order1)
        self.market.submit_order(order2)

        self.market.handle_pending_orders()

        # order1 should be executed, order2 should remain pending
        self.assertEqual(len(self.market.pending_orders), 1)
        self.assertIn(order2, self.market.pending_orders)
        self.assertEqual(self.market.current_position, 5.0)
        self.assertEqual(self.market.current_cash, 500.0)

    def test_record_state(self):
        # Set current market state
        self.market.current_price = 105.0
        self.market.current_time = 1234567890
        self.market.current_position = 5.0
        self.market.current_cash = 500.0

        self.market.record_state()

        self.assertEqual(len(self.market.records), 1)
        record = self.market.records[0]
        self.assertEqual(record.time, 1234567890)
        self.assertEqual(record.cash, 500.0)
        self.assertEqual(record.stock_price, 105.0)
        self.assertEqual(record.stock_position, 5.0)

    def test_full_order_lifecycle(self):
        # Initial state
        self.assertEqual(self.market.current_cash, 1000.0)
        self.assertEqual(self.market.current_position, 0.0)
        self.assertEqual(len(self.market.pending_orders), 0)

        # Update market data
        tick_data = MagicMock(spec=TickData)
        tick_data.price = 100.0
        tick_data.time = 1234567890
        self.market.update_market_data(tick_data)

        # Submit a buy order
        buy_order = MagicMock(spec=Order)
        buy_order.side = "buy"
        buy_order.price = 100.0
        buy_order.size = 5.0
        buy_order.time = 1234567890
        self.market.submit_order(buy_order)

        self.assertEqual(len(self.market.pending_orders), 1)

        # Handle pending orders
        self.market.handle_pending_orders()

        # Buy order should be executed
        self.assertEqual(len(self.market.pending_orders), 0)
        self.assertEqual(self.market.current_position, 5.0)
        self.assertEqual(self.market.current_cash, 500.0)

        # Record state
        self.market.record_state()
        self.assertEqual(len(self.market.records), 1)
        record = self.market.records[0]
        self.assertEqual(record.time, 1234567890)
        self.assertEqual(record.cash, 500.0)
        self.assertEqual(record.stock_price, 100.0)
        self.assertEqual(record.stock_position, 5.0)

        # Submit a sell order
        sell_order = MagicMock(spec=Order)
        sell_order.side = "sell"
        sell_order.price = 100.0
        sell_order.size = 2.0
        sell_order.time = 1234567900
        self.market.submit_order(sell_order)

        self.assertEqual(len(self.market.pending_orders), 1)

        # Update market data to allow sell order execution
        new_tick_data = MagicMock(spec=TickData)
        new_tick_data.price = 100.0
        new_tick_data.time = 1234567900
        self.market.update_market_data(new_tick_data)

        # Handle pending orders
        self.market.handle_pending_orders()

        # Sell order should be executed
        self.assertEqual(len(self.market.pending_orders), 0)
        self.assertEqual(self.market.current_position, 3.0)
        self.assertEqual(self.market.current_cash, 700.0)

        # Record final state
        self.market.record_state()
        self.assertEqual(len(self.market.records), 2)
        final_record = self.market.records[1]
        self.assertEqual(final_record.time, 1234567900)
        self.assertEqual(final_record.cash, 700.0)
        self.assertEqual(final_record.stock_price, 100.0)
        self.assertEqual(final_record.stock_position, 3.0)


if __name__ == "__main__":
    unittest.main()
