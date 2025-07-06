**Test Case Name**: Integration Test for Backtesting Manager, Agent Pool, and Data Storage

**Objective**:  
To validate the integration of the backtesting manager, agent pool, and data storage modules, ensuring that the historical backtesting process can be initiated and completed successfully, while the agent pool correctly manages the lifecycle of the RSI agent, and data storage provides the necessary historical data.

![UML](https://www.plantuml.com/plantuml/png/PP9DRkim38JtF0MNpq3U1RmeqfyM3T2YGHnqQp0MeGXn95MIYnw_b8KajhlbZ8OVAQvZef26Sm_9f9xnWRfJufYCrVXEbZI75TvhjWctplKh9AlmYH9XarmG6JSsSQ0k6MS1g9CgxZbA3S1BjEcC9tbzk0MtvnAC5F6GpZCRvLQjbrPP9JyJZoPAlEceRoTn9MPct3MRTamQG_HCKmBVnWAVPHK12Z3-lyiyDOR1pXhyQ_xKAX1dUL1ZatU8ZytdT7rY45tYQzpnro15VBtobnNcNCsfdSmbcASs8yOALBmqs0RtRHGl0-02GVxMH6Ol7pR6ZzzRTb41-T6i2_742j0xvmlKyqzt9AiPtwG4Y2W4rsTi5Uu5iU3lQop_PMmrEabVY4VdHr0S9ZOt28G8cVkwxEd5lHXhuf7LT2wvCxyOgkjTvELGIoUifPCSzYy0)

---

**Preconditions**:
- A CSV file with historical price data is available for the backtesting process.
- The backtesting manager is able to start the backtesting process and interact with the agent pool and data storage modules.
- The agent pool is capable of launching and closing the RSI agent upon request.
- The data storage module can provide historical tick data when queried by the backtesting module.

---

**Steps**:

1. **Service Invocation by Tester**:
   - The `Tester` invokes the `run_backtesting` service in the **backtesting manager** to begin the historical backtesting process.

2. **CSV Backtesting Execution**:
   - The backtesting manager starts the CSV backtesting process by invoking the **historical backtesting** module.
   - The historical backtesting module requests historical tick data by invoking the `get_historical_tick_datas` service from the **data storage** module.
   - The data storage module provides the historical tick data, which is then fed into the backtesting process.

3. **Agent Pool Service Request**:
   - The backtesting manager sends a request to the **agent pool** to launch the RSI agent by invoking the `start_agent` service.
   - The agent pool launches the **RSI agent**, which will interact with the backtesting process.

4. **Data Exchange Between Historical Backtesting and RSI Agent**:
   - During the backtesting process, the historical backtesting module sends tick data to the RSI agent via the `agent_in` topic.
   - The RSI agent processes the tick data and generates trading orders, which it sends back to the historical backtesting module via the `agent_out` topic.
   - This data exchange continues in a loop until the backtesting process reaches its conclusion.

5. **Backtesting Completion and Agent Shutdown**:
   - Once the backtesting is finished, the **historical backtesting** module sends a message to the backtesting manager.
   - The backtesting manager invokes the `close_agent` service to stop the RSI agent via the **agent pool**.
   - The agent pool stops the **RSI agent**, completing the testing process.

---

**Expected Results**:
- **Backtesting Manager**: The backtesting manager should successfully initiate the CSV backtesting process, request the RSI agent from the agent pool, and manage the shutdown of both modules after the backtesting process finishes.
- **Agent Pool**: The agent pool should launch the RSI agent when requested and stop it after receiving the shutdown request.
- **Historical Backtesting**: The historical backtesting module should interact correctly with the data storage and RSI agent, exchanging tick data and orders during the process.
- **Data Storage**: The data storage module should provide the required historical tick data when queried, ensuring the backtesting process has the necessary data to run.

This structured process ensures the system components integrate smoothly and verifies the correctness of the interactions between backtesting manager, agent pool, and data storage modules.