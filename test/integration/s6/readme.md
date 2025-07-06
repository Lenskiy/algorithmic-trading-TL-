**Test Case Name**: Integration Test for Data Storage and Risk Metrics

**Objective**:  
To validate the integration between the data storage and risk metrics modules, ensuring the data flow works correctly between the two modules when the risk assessment is requested.

![UML](https://www.plantuml.com/plantuml/png/NP31JWCn34Jl-OevqWC_i0TK2OIu8A4kzyWaLh6QtIsss_u-ZYAGuEfvyoRAtfpKBqjBBjuOJ-I4vxRTyC5-Orx7PVPDwSYWjU8WTi8hkqgnbAX4X7SsPqtdS4cHCwqE3Iml0mCPTFaJrz1c6zeUrt0D2-uV0fxmIArXPhrAOTpD4pQjhBbxTqcNZbMRSE2lI_Jm8jqcXTfuScySsVFdxpt7-2dtOTkbQZyxRxfTfV9_GKfxNcl_cMy0)

---

**Preconditions**:
- The data storage module contains historical tick data that can be requested by the risk metrics module.
- The risk metrics module is capable of requesting historical data from data storage to perform risk assessments.

---

**Steps**:

1. **Tester Calls Risk Metrics Service**:
   - The **Tester** invokes the `order_risk` service in the **risk_metrics** module to initiate a risk assessment.

2. **Risk Metrics Requests Historical Data**:
   - The risk metrics module sends a request to the **data_storage** module to retrieve historical tick data via the `get_historical_tick_datas` service.

3. **Data Storage Provides Historical Data**:
   - The **data_storage** module responds by providing the requested historical tick data to the **risk_metrics** module.

---

**Expected Results**:
- **Risk Metrics**: The risk metrics module should successfully request historical data from the data storage module and use it to perform a risk assessment.
- **Data Storage**: The data storage module should correctly provide the requested historical tick data to the risk metrics module.
