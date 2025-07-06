import rclpy
from rclpy.node import Node
from system_interface.srv import SetAgent, CloseAgent
from ..utils import ServiceServer, run_ros2_node_in_new_process


class AP(Node):
    def __init__(self):
        super().__init__("agent_pool")
        self.set_agent_server = ServiceServer(
            self, SetAgent, "set_agent", self.set_agent
        )
        self.close_agent_server = ServiceServer(
            self, CloseAgent, "close_agent", self.close_agent
        )
        self.count = 0

    def set_agent(self, agent_type, sub_name, pub_name):
        print("set_agent", agent_type, sub_name, pub_name)
        params = {
            "in": sub_name,
            "out": pub_name,
        }
        name = f"agent_{self.count}"
        self.count += 1

        run_ros2_node_in_new_process("backtesting", "fake_ag", params, name)
        return str(self.count - 1), True

    def close_agent(self, agent_id):
        print("close_agent", agent_id)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AP()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
