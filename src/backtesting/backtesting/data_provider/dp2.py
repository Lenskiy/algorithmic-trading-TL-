from system_interface.msg import TickData
from rclpy.node import Node
from .dp import DataProvider


class RealMarketDP(DataProvider):
    def __init__(self, node: Node):
        self.callback = None
        self.sub = node.create_subscription(
            TickData, "gateway_tick_data", self.handle_tick_data, 100
        )

    def register_callback(self, callback):
        self.callback = callback

    def handle_tick_data(self, tick_data: TickData):
        if self.callback:
            self.callback(tick_data)
