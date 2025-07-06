from threading import Thread
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from system_interface.msg import Order, TickData
from ..record import to_dataframe, calculate_max_drawdown, calculate_pnl
from ..market import Market
from ..data_provider import DataProvider
from ..utils import get_ros_param


class Backtesting(Node):
    data_provider: DataProvider

    @classmethod
    def run_node(cls, node_name):
        node = cls(node_name)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        Thread(target=node.run).start()

        try:
            while executor.get_nodes():
                executor.spin_once(0.1)
        finally:
            rclpy.shutdown()

    def __init__(self, node_name):
        super().__init__(node_name)
        self.init()

    def init(self):
        self.symbol = get_ros_param(self, "symbol", "AMD")
        self.exchange = get_ros_param(self, "exchange", "NASDAQ")
        self.agent_type = get_ros_param(self, "agent_type", "RSI")

        feed = get_ros_param(self, "feed", "agent/price")
        feedback = get_ros_param(self, "feedback", "agent/order")
        self.pub = self.create_publisher(TickData, feed, 100)
        self.sub = self.create_subscription(Order, feedback, self.submit_order, 100)

        self.jitter_type = get_ros_param(self, "jitter_type", "FIXED")
        self.jitter_value = get_ros_param(self, "jitter_value", 1000)

        initial_cash = get_ros_param(self, "budget", 1000.0)
        self.market = Market(initial_cash)

    def shutdown(self):
        # clear self
        self.destroy_node()
        executor = self.executor
        if executor is None:
            executor = rclpy.get_global_executor()
        executor.remove_node(self)

    def run(self):
        self.run_backtesting()
        self.analyze_records()
        self.shutdown()

    def run_backtesting(self):
        while self.data_provider.can_continue():
            data = self.data_provider.get_data()
            print(data)
            self.handle_tick_data(data)

    def handle_tick_data(self, tick_data: TickData):
        print("receive data", tick_data)
        self.market.update_market_data(tick_data)
        self.market.handle_pending_orders()
        self.pub.publish(tick_data)
        self.market.record_state()

    def submit_order(self, order: Order):
        # 固定时长的抖动，默认为1s
        if self.jitter_type == "FIXED":
            order.time += self.jitter_value
        self.market.submit_order(order)

    def print_data(self):
        # 输出数据
        pd = to_dataframe(self.market.records)
        pd.to_csv("result.csv")

    def analyze_records(self):
        try:
            pnl = calculate_pnl(self.market.records)
            max_drawdown = calculate_max_drawdown(self.market.records)
            print("pnl", pnl)
            print(f"final pnl {pnl[-1]:.3f}%")
            print(f"max_drawdown {max_drawdown:.3f}%")
        except Exception as e:
            print("analyze failed", e)
