**Test Case Name**: Integration Test for Gateway, Agent, Action Approval, Data Storage, and Risk Metrics (Real Trading)

**Objective**:  
To validate the integration between the gateway, agent, action approval, data storage, and risk metrics modules in a real trading scenario, ensuring the correct data flow from gateway to agent, with approval from action approval and risk metrics, and data exchange between data storage and risk metrics.

![UML](https://www.plantuml.com/plantuml/png/ZP5DJiCm48NtFiKiMwbS80lK2WHi4AZjtXh5nXtLFz5useZROyS9A3jMaTvydlSvtcT1bblmIaYyXMSKX1UVHlXA3AyeTiJl7HoSZL8-lL2AS1W6JdVqkuQV934wkmEC1ewKh_1kXQdFi3bQz71cD1JTLga1MQYd0QE0Qxi1CxXnDS5QLdDqwtngrpYtJmIk28E2EZUTEZPvDURYfaDpgsCEIhaH7XvBKmSdM_JFr5_RhJRdRd7KKb9TbszdtkJ_ykWgZpZp7smiJop5FWMAJgSQvK9Y9JbqxHy-NMnPlJbrC06woc-LoGlqoUbEni8RrGjJZxvvr1FEYhxyvdywfON3od_rNFNy323E6ijRgtt1Ry7_0000)

---

**Preconditions**:
- The gateway module is capable of retrieving live tick data and sending it directly to the agent.
- The agent is configured to process tick data using the RSI strategy and generate orders.
- The action approval module checks each order with the risk metrics module and approves the valid orders.
- The data storage module exchanges data with the risk metrics module for risk assessment.

---

**Steps**:

1. **Gateway Sends Tick Data to Agent**:
   - The **gateway** module retrieves live tick data and sends it to the **agent** module via the `gateway_tick_data` topic.

2. **Agent Sends Orders to Action Approval**:
   - The **agent** processes the tick data using the RSI strategy and generates trading orders.
   - The agent sends the generated orders to the **action_approval** module via the `incoming_orders` topic.

3. **Action Approval Requests Risk Check from Risk Metrics**:
   - The action approval module sends the order to the **risk_metrics** module via the `order_risk` service.
   - The risk metrics module performs a risk assessment and sends the approval status back to the action approval module.

4. **Action Approval Sends Approved Orders to Agent**:
   - If the risk metrics module approves the order, the action approval module sends the approved order back to the **agent** module.

5. **Data Exchange Between Data Storage and Risk Metrics**:
   - The **data_storage** module exchanges historical tick data and risk metrics data with the **risk_metrics** module as needed for the evaluation process.

---

**Expected Results**:
- **Gateway**: The gateway module should successfully send live tick data to the agent.
- **Agent**: The agent should process the tick data and generate orders, forwarding them to action approval.
- **Action Approval**: The action approval module should communicate with the risk metrics module to assess risk and only approve valid orders.
- **Risk Metrics**: The risk metrics module should evaluate the orders and provide accurate risk approval to the action approval module.
- **Data Storage**: The data storage module should correctly exchange data with the risk metrics module, ensuring smooth data flow for risk assessment.
