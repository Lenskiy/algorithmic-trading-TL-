import json
import subprocess
import rclpy
from rclpy.node import Node
from system_interface.srv import SetAgent, CloseAgent
from .utils import ServiceServer


class AP(Node):
    def __init__(self):
        super().__init__("agent_pool")
        self.agent_dict = {}
        self.count = 0

        self.s1 = ServiceServer(self, SetAgent, "set_agent", self.set_agent)
        self.s2 = ServiceServer(self, CloseAgent, "close_agent", self.close_agent)

    def set_agent(self, agent_type, input_topic, output_topic, agent_params_json="{}"):
        self.get_logger().info(
            f"Setting agent: {agent_type}, input: {input_topic}, output: {output_topic}"
        )
        agent_id = str(self.count)
        params_dict = json.loads(agent_params_json)

        # Convert params_dict to command line parameters
        param_list = []
        for key, value in params_dict.items():
            param_list.extend(["-p", f"{key}:={value}"])

        if agent_type == "RSI":
            entry_node = "ag1"
        elif agent_type == "MACD":
            entry_node = "ag2"
        elif agent_type == "BOLL":
            entry_node = "ag3"
        else:
            return "0", False

        # Start by command a process to run agent_node
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "agent_pool",
                entry_node,
                "--ros-args",
                "-p",
                f"input_topic:={input_topic}",
                "-p",
                f"output_topic:={output_topic}",
                "-p",
                f"agent_id:={agent_id}",
            ]
            + param_list
        )

        self.count += 1
        # Store the process and agent info
        self.agent_dict[agent_id] = {
            "agent_type": agent_type,
            "input_topic": input_topic,
            "output_topic": output_topic,
            "process": process,
        }
        self.get_logger().info(f"Created agent, id: {agent_id}")
        return agent_id, True

    def close_agent(self, agent_id):
        if agent_id not in self.agent_dict:
            self.get_logger().error(f"Agent id {agent_id} not found")
            return False

        process = self.agent_dict[agent_id]["process"]
        process.terminate()
        del self.agent_dict[agent_id]
        self.get_logger().info(f"Deleted agent, id: {agent_id}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
