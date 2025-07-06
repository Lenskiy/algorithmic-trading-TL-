from random import randint
import rclpy
from rclpy.node import Node
from system_interface.msg import TickData, Order
from ..utils import get_ros_param


class Agent(Node):
    def __init__(self):
        super().__init__("agent")
        agent_in = get_ros_param(self, "in", "agent/price")
        agent_out = get_ros_param(self, "out", "agent/order")
        self.sub = self.create_subscription(TickData, agent_in, self.handle_sub, 100)
        self.pub = self.create_publisher(Order, agent_out, 100)

    def handle_sub(self, tick_data: TickData):
        if randint(0, 3) < 1:
            order = Order()
            order.symbol = tick_data.symbol
            order.exchange = tick_data.exchange
            order.price = tick_data.price
            order.size = 1.0
            order.side = "buy"
            order.time = tick_data.time
            self.pub.publish(order)
            print(order)


def main(args=None):
    rclpy.init(args=args)
    node = Agent()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
