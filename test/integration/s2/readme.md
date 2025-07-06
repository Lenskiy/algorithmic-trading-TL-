**Test Case Name**: Integration Test for CSV Backtesting and MACD Agent

**Objective**:  
To validate the integration between the backtesting module and the agent module, ensuring that historical price data from the CSV is correctly processed by the MACD strategy to generate valid orders, and that the backtesting module correctly executes these orders and generates results.

![UML](https://www.plantuml.com/plantuml/png/JS-nhi8m38NXFKznndQuBz3XgWFC37H7RcAArUAWr2qyFWeHAFFvzLdkLwDastrn9hOm3cI4uylVI2V6oFPaLepaP-FLH2UG1j34QcXs8LXyY5zRvpGQ8-wSwhehwKKKpP1BN6F287xEXrfNmt__9UvmPWs_g9A_Up7ZPZLwIe5JwpvRjYgwYALmFMl8_xu1)

---

**Preconditions**:
- A CSV file with historical price data is available and properly formatted.
- The backtesting module is able to read from the CSV file and wrap the data as `TickData`.
- The agent module is configured to run the MACD strategy and produce trading orders based on the received tick data.

---

**Steps**:

1. **CSV Backtesting Initialization**:
   - Start the backtesting module and ensure it can correctly read the historical price data from the CSV file.
   - The price data must be formatted as `TickData` and sent via the `agent_in` to the agent module.

2. **Agent MACD Processing**:
   - The agent module should subscribe to the `agent_in` and receive the `TickData`.
   - The agent must process the tick data using the MACD strategy, producing valid buy/sell signals.
   - The agent should convert the signals into `Order` objects and send them via the `agent_out` to the backtesting module.

3. **Backtesting Order Execution**:
   - The backtesting module should receive the `Order` objects from the `agent_out`.
   - The backtesting module must simulate the execution of these orders based on the tick data and historical price movements.
   - The backtesting module should generate the final results, including performance metrics such as profit, loss, and the effectiveness of the MACD strategy.

---

**Expected Results**:
- **CSV Backtesting**: The module should read the CSV file without errors and successfully send all tick data to the agent module via the `agent_in`.
- **Agent MACD**: The agent should correctly process the tick data, generate appropriate `Order` objects, and send them back to the backtesting module via `agent_out`.
- **Final Results**: The backtesting module should accurately simulate the orders and produce meaningful results, including the performance of the MACD strategy over the given historical data.

