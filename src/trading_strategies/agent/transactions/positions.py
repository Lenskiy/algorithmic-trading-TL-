from src.trading_strategies.agent.transactions.position import Position

"""
@author: Yifan Xiao
@author: Huanjie Zhang
"""
class Positions:
    def __init__(self, position: Position, quantity: int):
        self.position = position
        self.quantity = quantity

    def __str__(self):
        return f"{self.position.__str__()}, {str(self.quantity)} asset(s)"


class ShortPositions(Positions):
    def __init__(self, quantity: int):
        super().__init__(Position.SHORT, quantity)


class LongPositions(Positions):
    def __init__(self, quantity: int):
        super().__init__(Position.LONG, quantity)
