from datetime import datetime
from enum import Enum


class RatePeriod(Enum):
    ONE_MONTH = "1-month"
    THREE_MONTH = "3-month"
    SIX_MONTH = "6-month"
    FIVE_YEAR = "5-year"
    TEN_YEAR = "10-year"

    def __hash__(self):
        return self.value.__hash__()


class RiskFree:

    def __init__(self, value: float, period: RatePeriod, date: datetime):
        self.value = value
        self.period = period
        self.date = date

    def __hash__(self):
        return 5 * self.value.__hash__() + 7 * self.period.__hash__() + 11 * self.date.__hash__()
