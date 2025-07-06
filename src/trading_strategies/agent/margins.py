from datetime import datetime
from typing import List, Tuple, Dict

import pandas as pd


class Margins:
    def __init__(self):
        self._margins: List[Tuple[datetime, float]] = list()

    def append_margin(self, date: datetime, margin: float, initial=False):
        if not initial:
            margin = max(margin, self.peak_last()[1])
        self._margins.append(tuple((date, margin)))

    def peak_last(self) -> Tuple[datetime, float]:
        if len(self._margins) == 0:
            return datetime(year=2000, month=1, day=1), 0
        return self._margins[len(self._margins) - 1]

    def get_margins(self):
        return self._margins

    def to_dataframe(self):
        return pd.DataFrame(self._margins, columns=["Date", "Margin"])

    def max_margin(self):
        return max(x[1] for x in self._margins)
