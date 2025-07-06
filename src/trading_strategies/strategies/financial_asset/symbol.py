

class Symbol:
    def __init__(self, symbol: str):
        self._symbol = symbol

    @property
    def symbol(self) -> str:
        return self._symbol

    def __eq__(self, other):
        if not isinstance(other, Symbol):
            return False
        return self._symbol.__eq__(other._symbol)

    def __hash__(self):
        return self.symbol.__hash__()

    def __str__(self):
        return self._symbol
