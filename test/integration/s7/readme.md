**Test Case Name**: Integration Test for Gateway and Data Storage

**Objective**:  
To validate the integration between the gateway and data storage modules, ensuring that the gateway can fetch tick data and send it to the data storage module, and that the data storage module can store the received data in a Parquet file.

![UML](https://www.plantuml.com/plantuml/png/LOynZi8m44Lxd-ANkyLU8CKg1GejKj97GpmoLXmds1DPt9u12n3bwBt_f6cpK99z2KQy1CQ1XD25jU3CKfWZZYHSw0QAjj9UraIEZTbqwIU_KHIubvNXoXUnkXbptJompSQuWhz_5HjqBDC5Wv_cPxmITHhq7Eq7u6Tei52QMKQhzw-n-MrgdAlb8qwKhZiBEX_Oj1hHv-u0)

---

**Preconditions**:
- The gateway module is connected to a data source to fetch tick data.
- The data storage module is capable of receiving data via the `market_data` topic and storing it in Parquet format.

---

**Steps**:

1. **Gateway Fetches Tick Data**:
   - The **gateway** module retrieves tick data from its data source.

2. **Gateway Sends Data to Data Storage**:
   - The gateway module sends the fetched tick data to the **data_storage** module via the `market_data` topic.

3. **Data Storage Stores the Data**:
   - The **data_storage** module receives the tick data and stores it in a Parquet file.

---

**Expected Results**:
- **Gateway**: The gateway module should successfully fetch tick data from its source and send it via the `market_data` topic.
- **Data Storage**: The data storage module should correctly receive the tick data and store it in a Parquet file without any errors.
