**Test Case Name**: Integration Test for Backtesting, Agent, Action Approval, and Data Storage Modules

**Objective**:  
To validate the integration between the backtesting, agent, action approval, and data storage modules, ensuring that the data flow works correctly through all stages, from data storage to backtesting, agent processing, order approval, and back to backtesting.

![UML](https://www.plantuml.com/plantuml/png/NP7DJiCm48JlVeezKgcyG0weGO0uW42qTsjZM-wghXti3V3uhEF-ACqXYcPcludNcyAOUJe6nSJ1mHComaj8lq0H7j4Ss1McwDVGUnTv3HWjz8OfHUZ7CQSV3F2Ux1HSKMeK5XaQCJ8Crs700WClNAoqggDVwZDQunbT4xX8M6JKXLCMHBMjz0uCzm_orS6N-vwa55wfHiXWQ4UwstRW7UlzHTT2FEXAAojbImKCtC6dz99rjm2bGafZOpt_cJr6QNyJIO4yhHzz1riNBUo4t5ftNJkffhYQa-Zd99wI7zcwFCSecdIKNUbKfN9Au_ctsgvTNV_j93RBNYR5LXjvoQt-0m00)

---

**Preconditions**:
- The backtesting module can request historical tick data from data storage and send it to the agent.
- The agent is configured to process tick data using the RSI strategy and generate orders.
- The action approval module is set to approve all incoming orders and forward them to backtesting.
- The mock risk metrics module is not involved in this test case but remains part of the overall structure.

---

**Steps**:

1. **Backtesting Module Requests Historical Data**:
   - The **backtesting** module requests historical tick data from the **data_storage** module via the `get_historical_tick_datas` service.
   - The **data_storage** module provides the historical data to backtesting.

2. **Backtesting Sends TickData to Agent**:
   - The backtesting module sends tick data to the **agent** module via the `agent_in` topic.
   - The **agent** processes the tick data using the RSI strategy and generates trading orders.

3. **Agent Sends Orders to Action Approval**:
   - The agent sends the generated orders to the **action_approval** module via the `incoming_orders` topic.
   - The action approval module approves all incoming orders (since it is configured to approve everything for this test).

4. **Action Approval Sends Approved Orders to Backtesting**:
   - The action approval module sends the approved orders back to the **backtesting** module via the `approved_orders` topic.

---

**Expected Results**:
- **Data Storage**: The data storage module should provide the correct historical tick data upon request.
- **Backtesting**: The backtesting module should send the tick data to the agent and correctly receive approved orders back from the action approval module.
- **Agent**: The agent should process the tick data and generate orders based on the RSI strategy, forwarding them to the action approval module.
- **Action Approval**: The action approval module should approve all orders and send them back to the backtesting module for final processing.