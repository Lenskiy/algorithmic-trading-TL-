from typing import Optional
from rclpy.node import Node
from system_interface.msg import TickData, Order
from ..utils import get_ros_param


class Agent(Node):
    def __init__(self, node_name="agent_node"):
        super().__init__(node_name)
        self.init()

    def init(self):
        input_topic = get_ros_param(self, "input_topic", "agent_in")
        output_topic = get_ros_param(self, "output_topic", "agent_out")

        self.sub = self.create_subscription(TickData, input_topic, self.handle_data, 10)
        self.pub = self.create_publisher(Order, output_topic, 10)
        self.get_logger().info(
            f"{self.__class__.__name__} {input_topic} {output_topic}"
        )

    def on_price(self, tick_data: TickData) -> Optional[Order]:
        raise NotImplementedError

    def handle_data(self, tick_data: TickData):
        order = self.on_price(tick_data)
        if order:
            print(order)
            self.pub.publish(order)
