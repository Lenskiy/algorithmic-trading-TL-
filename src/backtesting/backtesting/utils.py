import subprocess
from typing import TypeVar
from rclpy.node import Node
from rclpy.executors import await_or_execute
import re
from rclpy.callback_groups import ReentrantCallbackGroup

T = TypeVar("T")


def get_run_ros2_node_cmds(
    package_name: str,
    module_name: str,
    params: dict = {},
    rename: str = "",
):
    cmds = ["ros2", "run", package_name, module_name]

    if params:
        cmds.append("--ros-args")

        # 将字典转换为命令行参数格式
        for key, value in params.items():
            cmds.append("-p")
            cmds.append(f"{key}:={value}")

    # 重命名
    if rename:
        cmds.append("--remap")
        cmds.append(f"__node:={rename}")
 
    print(" ".join(cmds))
    return cmds


def run_ros2_node_in_new_process(
    package_name: str,
    module_name: str,
    params: dict = {},
    rename: str = "",
):
    cmds = get_run_ros2_node_cmds(package_name, module_name, params, rename)

    # 启动新进程，传递功能节点的名称和参数
    subprocess.Popen(cmds)
    print(f"Successfully started {package_name} {module_name} with params: {params}")


def get_ros_param(node: Node, name: str, default: T) -> T:
    node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    return value  # type: ignore


def get_all_topic_names(node: Node):
    """获取所有的 topic 名称和类型"""
    topics = node.get_topic_names_and_types()
    topic_names = [name[1:] for name, types in topics]
    return topic_names


def avoid_duplicate(name: str, names: list[str]) -> str:
    """自动避让已经使用过的名字"""
    # 如果名字不在names中，直接返回name
    if name not in names:
        return name

    # 动态构建正则表达式，匹配以name开头并以数字结尾的字符串
    pattern = re.compile(r"^" + re.escape(name) + r"(\d+)$")

    # 找到所有匹配name的项
    similar_names = [n for n in names if pattern.match(n)]

    # 如果没有匹配的名字，直接返回name + "2"
    if not similar_names:
        return name + "2"

    # 查找最大的数字后缀
    max_num = 1
    for n in similar_names:
        match = pattern.search(n)
        if match:
            max_num = max(max_num, int(match.group(1)))

    # 返回name加上最大的后缀+1
    return name + str(max_num + 1)


def _get_keys(ros2_type):
    return tuple(ros2_type.get_fields_and_field_types().keys())


def print_info(node: Node, *args):
    s = " ".join(str(arg) for arg in args)
    node.get_logger().info(s)


class ServiceServer:
    def __init__(self, node: Node, service_type, service_name, callback):
        """这里请保证，callback的参数顺序，和service_type定义的顺序一致"""
        self.node = node
        self.service_type = service_type
        self.service_name = service_name
        self.callback = callback

        # 获取键名
        self.request_keys = _get_keys(service_type.Request())
        self.response_keys = _get_keys(service_type.Response())

        # 存储服务端实例
        self.server = node.create_service(
            service_type,
            service_name,
            self.handle,
            callback_group=ReentrantCallbackGroup(),
        )
        node.get_logger().info(f"Created service {service_name}")

    async def handle(self, request, response):
        # 提取请求字段值，按顺序传递给回调函数
        args = tuple(getattr(request, key) for key in self.request_keys)
        result = await await_or_execute(self.callback, *args)

        # 特殊设置
        if len(self.response_keys) == 1:
            key = self.response_keys[0]
            setattr(response, key, result)
            return response

        # 将结果按顺序组装到 response 对应字段并返回
        for i, key in enumerate(self.response_keys):
            val = result[i]
            setattr(response, key, val)
        return response


class ServiceClient:
    def __init__(self, node: Node, service_type, service_name):
        self.node = node
        self.service_type = service_type
        self.service_name = service_name

        # 获取键名
        self.request_keys = _get_keys(service_type.Request())
        self.response_keys = _get_keys(service_type.Response())

        self.client = None

    def init(self):
        # 存储客户端实例
        self.client = self.node.create_client(self.service_type, self.service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f"Waiting for service {self.service_name}...")
        self.node.get_logger().info(f"Connected to service {self.service_name}")

    def call(self, *args):
        """发送请求并处理响应
        1. 请保证args的参数顺序，和service_type定义的顺序一致
        2. 请在新的线程中调用该方法，以免阻塞ros2的事件循环"""

        if not self.client:
            self.init()
        assert self.client

        # 新建请求
        request = self.service_type.Request()

        info = f"[{self.service_name}]"
        # 根据请求字段值，按顺序构建request
        for i, key in enumerate(self.request_keys):
            val = args[i]
            info += f" {key}: {val},"
            setattr(request, key, val)
        self.node.get_logger().info(info[:-1])

        response = self.client.call(request)

        # 特殊设置
        if len(self.response_keys) == 1:
            key = self.response_keys[0]
            return getattr(response, key)

        # 将结果按顺序组装到 response 对应字段并返回
        result = tuple(getattr(response, key) for key in self.response_keys)
        return result
