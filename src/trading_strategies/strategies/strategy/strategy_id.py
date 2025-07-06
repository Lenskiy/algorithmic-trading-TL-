class StrategyId:

    def __init__(self, id: str):
        self._id = id

    def get_id(self):
        return self._id

    def __eq__(self, other):
        if not isinstance(other, StrategyId):
            return False
        return self._id == other._id

    def __hash__(self):
        return self._id.__hash__()

    def __str__(self):
        return self._id
