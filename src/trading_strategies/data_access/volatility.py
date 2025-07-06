from datetime import datetime
from enum import Enum


class VolatilityType(Enum):
    GARCH = "Garch"
    SIMPLE = "Standard_Deviation"

    def __hash__(self):
        return self.value.__hash__()


class Volatility:

    def __init__(self, value: float, volatility_type: VolatilityType):
        self.value = value
        self.type = volatility_type
        # self.start_date = start_date
        # self._end_date = end_date

    # def __hash__(self):
    #     return (2 * self.value.__hash__() + 3 * self.type.__hash__() +
    #             5 * self.start_date.__hash__() + 7 * self._end_date.__hash__())


class EmptyVolatility(Volatility):
    # in the case, it is unable to calculate var, mostly because of not enough data.

    def __init__(self):
        value = 0
        volatility_type = VolatilityType.SIMPLE
        super().__init__(value, volatility_type)
