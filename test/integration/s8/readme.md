**Test Case Name**: Integration Test for Gateway, Backtesting, Agent, Action Approval, Data Storage, and Risk Metrics

**Objective**:  
To validate the integration of all modules, ensuring the correct data flow from the gateway to backtesting and agent, with approval from action approval and risk metrics, while also verifying the data exchange between data storage and risk metrics.

![UML](https://www.plantuml.com/plantuml/png/ZPB1JiCm44Jl_eezjbBuWHnG2mYu813jtPh6nhMQs96z9UBliUEaRJXnYZJprfxPU1CPWSzTAzXog-a937fk_K1VFj0BM0_ugUW1pObrPEjCHLkZ7QTFmzOxslPzyDzegpAxOnzWT4LmYZvjFD6Rvc2RA4IFmBQnFHoJAISJ8fbXuHokLsNtm0iN-UwimoprCOp456Oai20K63AMTDbMSM6755Ls9MgsGoU46UZkFYMeQQTJcxrjJgNaQb_F3IGdLUR3rnEludzvSU0y07CpCFEbgRHkBM162XFr7fGE8s9TuplhZFHPYeb49aDNbyq_7dLAiDhLD08oTrmB5MVe8-tAAaslDk_5DcXBt3nXDClZHR-cJhU4xgRcyYUFynC0b1YN5oA0UPz5qlcH_2s5OgbBgKtoprtx2m00)

---

**Preconditions**:
- The gateway module is capable of retrieving tick data and sending it to backtesting.
- The backtesting module can forward tick data to the agent and receive orders back from the action approval module.
- The agent processes tick data using the RSI strategy and generates orders.
- The action approval module checks each order with the risk metrics module and approves the valid orders.
- The data storage module exchanges data with the risk metrics module as needed.

---

**Steps**:

1. **Gateway Sends Tick Data to Backtesting**:
   - The **gateway** module retrieves tick data and sends it to the **backtesting** module via the `gateway_tick_data` topic.

2. **Backtesting Sends Tick Data to Agent**:
   - The **backtesting** module sends the tick data to the **agent** module via the `agent_in` topic.
   - The **agent** processes the tick data using the RSI strategy and generates orders.

3. **Agent Sends Orders to Action Approval**:
   - The **agent** sends the generated orders to the **action_approval** module via the `incoming_orders` topic.

4. **Action Approval Requests Risk Check from Risk Metrics**:
   - The action approval module sends the order to the **risk_metrics** module via the `order_risk` service.
   - The risk metrics module performs a risk assessment and sends the approval status back to the action approval module.

5. **Action Approval Sends Approved Orders to Backtesting**:
   - If the risk metrics module approves the order, the action approval module sends the approved order back to the **backtesting** module via the `approved_orders` topic.

6. **Data Exchange Between Data Storage and Risk Metrics**:
   - The **data_storage** module exchanges historical tick data and risk metrics data with the **risk_metrics** module as needed for the evaluation process.

---

**Expected Results**:
- **Gateway**: The gateway module should successfully send tick data to backtesting.
- **Backtesting**: The backtesting module should correctly forward the tick data to the agent and process the approved orders from action approval.
- **Agent**: The agent should process tick data and generate orders, forwarding them to action approval.
- **Action Approval**: The action approval module should communicate with the risk metrics module to assess risk and only approve valid orders.
- **Risk Metrics**: The risk metrics module should evaluate the orders and provide accurate risk approval to the action approval module.
- **Data Storage**: The data storage module should correctly exchange data with the risk metrics module, ensuring smooth data flow for risk assessment.
