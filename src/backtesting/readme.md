## Backtesting scenarios

### Historical Data Backtesting

Get historical data from Data Provider Module, and feed them one by one to Agent Node.

**ros parameters**

| Name           | Type    | Description                                                              | Default Value          |
| -------------- | ------- | ------------------------------------------------------------------------ | ---------------------- |
| `symbol`       | `str`   | The trading symbol, such as 'AMD' or other financial instruments.        | `"AMD"`                |
| `exchange`     | `str`   | The name of the exchange, such as 'NASDAQ', etc.                         | `"NASDAQ"`             |
| `agent_type`   | `str`   | The type of the agent, representing the strategy used, such as 'RSI'.    | `"RSI"`                |
| `feed`         | `str`   | The topic for price feed, used for publishing price data.                | `"agent/price"`        |
| `feedback`     | `str`   | The topic for order feedback, used for receiving order information.      | `"agent/order"`        |
| `jitter_type`  | `str`   | The type of jitter applied to the agent's actions (e.g., "FIXED").       | `"FIXED"`              |
| `jitter_value` | `int`   | The value of the jitter in milliseconds, affecting the agent's timing.   | `1000`                 |
| `budget`       | `float` | The initial cash balance, representing the starting cash in the account. | `1000.0`               |
| `start_time`   | `int`   | The start time for the simulation, represented as a Unix timestamp.      | `1726012266_000`       |
| `end_time`     | `int`   | The end time for the simulation, calculated as start time + 600,000 ms.  | `start_time + 600_000` |

**usage example**

`zsh test/integration/s5.zsh`

### Real Market Backtesting

Get real-world price from Gateway Module, and forward it to Agent Node.

**ros parameters**

| Name           | Type    | Description                                                              | Default Value   |
| -------------- | ------- | ------------------------------------------------------------------------ | --------------- |
| `symbol`       | `str`   | The trading symbol, such as 'AMD' or other financial instruments.        | `"AMD"`         |
| `exchange`     | `str`   | The name of the exchange, such as 'NASDAQ', etc.                         | `"NASDAQ"`      |
| `agent_type`   | `str`   | The type of the agent, representing the strategy used, such as 'RSI'.    | `"RSI"`         |
| `feed`         | `str`   | The topic for price feed, used for publishing price data.                | `"agent/price"` |
| `feedback`     | `str`   | The topic for order feedback, used for receiving order information.      | `"agent/order"` |
| `jitter_type`  | `str`   | The type of jitter applied to the agent's actions (e.g., "FIXED").       | `"FIXED"`       |
| `jitter_value` | `int`   | The value of the jitter in milliseconds, affecting the agent's timing.   | `1000`          |
| `budget`       | `float` | The initial cash balance, representing the starting cash in the account. | `1000.0`        |
| `duration`     | `float` | The duration of the trading simulation in seconds.                       | `600.0`         |

**usage example**

`zsh test/integration/s6.zsh`

### Data Frame Backtesting

Get historical data from local data file, and feed them one by one to Agent Node.

**ros parameters**

| Name           | Type    | Description                                                                      | Default Value                                              |
| -------------- | ------- | -------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| `symbol`       | `str`   | The trading symbol, such as 'AMD' or other financial instruments.                | `"AMD"`                                                    |
| `exchange`     | `str`   | The name of the exchange, such as 'NASDAQ', etc.                                 | `"NASDAQ"`                                                 |
| `agent_type`   | `str`   | The type of the agent, representing the strategy used, such as 'RSI'.            | `"RSI"`                                                    |
| `feed`         | `str`   | The topic for price feed, used for publishing price data.                        | `"agent/price"`                                            |
| `feedback`     | `str`   | The topic for order feedback, used for receiving order information.              | `"agent/order"`                                            |
| `jitter_type`  | `str`   | The type of jitter applied to the agent's actions (e.g., "FIXED").               | `"FIXED"`                                                  |
| `jitter_value` | `int`   | The value of the jitter in milliseconds, affecting the agent's timing.           | `1000`                                                     |
| `budget`       | `float` | The initial cash balance, representing the starting cash in the account.         | `1000.0`                                                   |
| `filepath`     | `str`   | The file path to the data, which the agent may use for reference or backtesting. | `"src/trading_strategies/data/sp500_adj_close_prices.csv"` |

**usage example**

`zsh test/integration/s2.zsh`

## Backtesting Performance Metrics

- `PnL` (Profit and Loss)
- `Max Drawdown`
