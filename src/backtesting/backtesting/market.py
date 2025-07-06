from .record import Record
from system_interface.msg import Order, TickData


class Market:
    def __init__(self, initial_cash: float):
        self.current_price = 0.0
        self.current_time = 0
        self.current_position = 0.0
        self.current_cash = initial_cash
        self.pending_orders: list[Order] = []
        self.records: list[Record] = []

    def update_market_data(self, tick_data: TickData):
        self.current_price = tick_data.price
        self.current_time = tick_data.time

    def submit_order(self, order: Order):
        print("order received", order)
        self.pending_orders.append(order)

    def execute_order(self, order: Order):
        # limit order
        if order.side == "buy" and order.price >= self.current_price:
            self.current_position += order.size
            self.current_cash -= order.size * self.current_price
            print("order executed", order)
            return True

        elif order.side == "sell" and order.price <= self.current_price:
            self.current_position -= order.size
            self.current_cash += order.size * self.current_price
            print("order executed", order)
            return True

        return False

    def handle_pending_orders(self):
        # 提取出时间合适的order
        executing_orders = [
            order for order in self.pending_orders if order.time <= self.current_time
        ]

        # 提取出成功执行的order
        executed_orders = [
            order for order in executing_orders if self.execute_order(order)
        ]

        # remove successful orders
        for order in executed_orders:
            self.pending_orders.remove(order)

    def record_state(self):
        record = Record(
            self.current_time,
            self.current_cash,
            self.current_price,
            self.current_position,
        )
        self.records.append(record)
