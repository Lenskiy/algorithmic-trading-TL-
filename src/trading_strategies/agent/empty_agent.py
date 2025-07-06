from src.trading_strategies.agent.agent import Agent


class EmptyAgent(Agent):
    def __init__(self, strategies=None):
        if strategies is None:
            strategies = dict()
        super().__init__(strategies)
