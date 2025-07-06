import subprocess
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from system_interface.srv import RunBacktesting, SetAgent, CloseAgent
from .utils import ServiceClient, ServiceServer, get_run_ros2_node_cmds


class BacktestingManager(Node):
    @classmethod
    def run_node(cls, node_name):
        node = cls(node_name)
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            while executor.get_nodes():
                executor.spin_once(0.1)
        finally:
            rclpy.shutdown()

    def __init__(self, node_name):
        super().__init__(node_name)
        self.s1 = ServiceServer(
            self, RunBacktesting, "run_backtesting", self.run_backtesting
        )
        self.c1 = ServiceClient(self, SetAgent, "set_agent")
        self.c2 = ServiceClient(self, CloseAgent, "close_agent")
        self.count = 0
        self.data = {}

    def run_backtesting(self, bt_type: str, bt_params_json: str):
        node_name = f"backtesting_{self.count}"
        self.count += 1
        package_name = "backtesting"

        if bt_type == "bt1":
            module_name = "bt1"
            params = {
                "agent_type": "RSI",
                "feed": "agent_in",
                "feedback": "agent_out",
            }

            # 启动agent
            agent_id, done = self.c1.call("RSI", "agent_in", "agent_out")

            # 启动backtesting
            cmds = get_run_ros2_node_cmds(package_name, module_name, params, node_name)
            subprocess.run(cmds)

            # 关闭agent
            self.c2.call(agent_id)
            return True, ""

        if bt_type == "bt2":
            module_name = "bt2"
            params = {
                "agent_type": "RSI",
                "feed": "agent_in",
                "feedback": "agent_out",
            }
            # 启动agent
            agent_id, done = self.c1.call("RSI", "agent_in", "agent_out")

            # 启动backtesting
            cmds = get_run_ros2_node_cmds(package_name, module_name, params, node_name)
            subprocess.run(cmds)

            # 关闭agent
            self.c2.call(agent_id)
            return True, ""

        if bt_type == "bt3":
            module_name = "bt3"
            params = {
                "agent_type": "RSI",
                "feed": "agent_in",
                "feedback": "agent_out",
            }
            # 启动agent
            agent_id, done = self.c1.call("RSI", "agent_in", "agent_out")

            # 启动backtesting
            cmds = get_run_ros2_node_cmds(package_name, module_name, params, node_name)
            subprocess.run(cmds)

            # 关闭agent
            self.c2.call(agent_id)
            return True, ""
        
        if bt_type == "bt4":
            module_name = "bt3"
            params = {
                "agent_type": "RSI",
                "feed": "agent_in",
                "feedback": "approved_orders",
            }
            # 启动agent
            agent_id, done = self.c1.call("RSI", "agent_in", "incoming_orders")

            # 启动backtesting
            cmds = get_run_ros2_node_cmds(package_name, module_name, params, node_name)
            subprocess.run(cmds)

            # 关闭agent
            self.c2.call(agent_id)
            return True, ""

        return False, ""


def main(args=None):
    rclpy.init(args=args)
    BacktestingManager.run_node("btm")


if __name__ == "__main__":
    main()
