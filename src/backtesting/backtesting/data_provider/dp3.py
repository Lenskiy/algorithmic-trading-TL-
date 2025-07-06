import pandas as pd
from system_interface.msg import TickData
from time import mktime, sleep
from .dp import DataProvider


class LocalFileDP(DataProvider):
    def __init__(
        self, filepath: str, symbol: str, exchange: str, data_num: int, time_gap: float
    ):
        self.tick_datas: list[TickData] = []
        self.index = 0
        self.data_num = data_num
        self.time_gap = time_gap
        self.load_data(filepath, symbol, exchange)

    def load_data(self, filepath: str, symbol: str, exchange: str):
        df = pd.read_csv(filepath)
        df["Date"] = pd.to_datetime(df["Date"])  # 首先将Date列转换为datetime格式
        df["Date"] = df["Date"].apply(
            lambda x: int(mktime(x.timetuple()) * 1000)
        )  # 转换为以毫秒为单位的时间戳

        # 取出指定列
        df = df[["Date", symbol]]
        df = df.dropna()

        tick_datas = []
        for index, row in df.iterrows():
            time = int(row["Date"])

            tick_data = TickData()
            tick_data.symbol = symbol
            tick_data.exchange = exchange
            tick_data.size = 100.0
            tick_data.is_buy = True
            tick_data.time = time
            tick_data.price = float(row[symbol])
            tick_datas.append(tick_data)

        self.tick_datas = tick_datas

    def get_data(self) -> TickData:
        sleep(self.time_gap)
        tick_data = self.tick_datas[self.index]
        self.index += 1
        return tick_data

    def can_continue(self) -> bool:
        return self.index < min(len(self.tick_datas), self.data_num)
