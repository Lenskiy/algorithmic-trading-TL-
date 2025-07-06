**Test Case Name**: Integration Test for Backtesting, Agent, Action Approval, Risk Metrics, and Data Storage Modules

**Objective**:  
To validate the integration between the backtesting, agent, action approval, risk metrics, and data storage modules, ensuring that the data flow works correctly through all stages, including the new risk check step, and that data flows smoothly between the data storage and risk metrics modules.

![UML](https://www.plantuml.com/plantuml/png/fLBBJiCm4BpxA_P8fV83EQ2M1-901MfxjpQsbLf9x61lWSzdxKMfKeaJFJJHp6XcTxARJ9ZeF0wAF0qE7f4GdeVu1Jx0FPgUN2OVkZLiEnU87uPy3B2TfXG_SLW31WjxdtlOEKhUPDX5UnvSLcfYPs_yX86WlLW1PcXfmMBnBZXsIxo6QVmDgmfSA18dwr6I2vt61MsHK6UAYHCARNcibk3sZjyQsBkFCyy4Bxv8lC51zhyvDD0vqgSPr-pOw-AMLyhcOj1I0-yybhVksa2f8SP9l9w-pGa3fxyof81_aezT0mSNB1pPMLAFZHIXVLXL9NPLYQA4jsHTgZ8VJ1ovIXqBb4MDMDHfREHpF9oSwUigLQHBKo9CuxyA41ER9Utl4YvHY3BJV3Npl5AZmAwSqlMcA9ITDsLQgGt_bvlz0G00)

---

**Preconditions**:
- The backtesting module can request historical tick data from data storage and send it to the agent.
- The agent is configured to process tick data using the RSI strategy and generate orders.
- The action approval module can communicate with the risk metrics server to determine whether orders should be approved.
- The risk metrics module can provide risk assessments for incoming orders via the `order_risk` service.

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

4. **Action Approval Requests Risk Check from Risk Metrics**:
   - The action approval module sends the order to the **risk_metrics** module via the `order_risk` service.
   - The risk metrics module evaluates the risk of the order and responds with an approval or rejection.

5. **Action Approval Sends Approved Orders to Backtesting**:
   - If the risk metrics module approves the order, the action approval module sends the approved order back to the **backtesting** module via the `approved_orders` topic.

6. **Risk Metrics and Data Storage Data Exchange**:
   - The **data_storage** module and **risk_metrics** module exchange data in a bidirectional manner for continuous risk assessment and data updating.

---

**Expected Results**:
- **Data Storage**: The data storage module should provide the correct historical tick data upon request and exchange necessary data with the risk metrics module.
- **Backtesting**: The backtesting module should send the tick data to the agent and correctly receive approved orders back from the action approval module.
- **Agent**: The agent should process the tick data and generate orders based on the RSI strategy, forwarding them to the action approval module.
- **Action Approval**: The action approval module should communicate with the risk metrics module to approve or reject incoming orders, and only forward approved orders back to backtesting.
- **Risk Metrics**: The risk metrics module should evaluate the risk for incoming orders and provide an accurate approval or rejection response.