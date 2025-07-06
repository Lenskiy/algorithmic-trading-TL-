import json
import datetime
from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from system_interface.msg import TickData
from websocket import WebSocketApp

# BitMEX WebSocket URL
BITMEX_WS_URL = "wss://www.bitmex.com/realtime"
HEARTBEAT_INTERVAL = 5


class FakeGTW(Node):
    def __init__(self):
        super().__init__("fake_gateway")
        self.gateway_tick_data_pub = self.create_publisher(
            TickData, "gateway_tick_data", 100
        )
        Thread(target=self.run_ws).start()

    def run_ws(self):
        # 创建 WebSocket 连接
        self.ws = WebSocketApp(
            BITMEX_WS_URL,
            on_message=self.on_message,
            on_open=self.on_open,
            on_close=self.on_close,
            on_error=self.on_error,
        )

        WebSocketApp.run_forever(
            self.ws,
            ping_interval=HEARTBEAT_INTERVAL,
            ping_timeout=1,
        )

    # 回调函数，连接打开时调用
    def on_open(self, ws):
        print("WebSocket connection opened")
        # 订阅 trade 主题
        subscribe_message = json.dumps({"op": "subscribe", "args": ["trade:XBTUSD"]})
        ws.send(subscribe_message)

    # 回调函数，连接关闭时调用
    def on_close(self, ws, a, b):
        print("WebSocket connection closed")
        self.shutdown()

    # 回调函数，遇到错误时调用
    def on_error(self, ws, error):
        print(f"Error occurred: {error}")

    # 回调函数，接收消息
    def on_message(self, ws, message):
        data = json.loads(message)
        print(f"Received message: {data}")
        data = data.get("data", [])
        for d in data:
            # 解析时间字符串，并转换为datetime对象
            dt = datetime.datetime.strptime(d["timestamp"], "%Y-%m-%dT%H:%M:%S.%fZ")
            time = int(dt.timestamp() * 1000)

            tick_data = TickData()
            tick_data.symbol = d["symbol"]
            tick_data.exchange = "BITMEX"
            tick_data.price = d["price"]
            tick_data.size = float(d["size"])
            tick_data.is_buy = False if d["side"] == "Sell" else True
            tick_data.time = time
            self.gateway_tick_data_pub.publish(tick_data)

    def shutdown(self):
        # clear self
        self.destroy_node()
        executor = self.executor
        if executor is None:
            executor = rclpy.get_global_executor()
        executor.remove_node(self)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGTW()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while executor.get_nodes():
            executor.spin_once(0.1)  # type: ignore
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
