# Contents
- [Data Manager](#data-Manager) 
- [Agent Pool](#Agent-Pool) 
- [Portfolio Manager](#Portfolio-Manager) 
- [Backtesting](#Backtesting) 
- [Action Approval](#Action-Approval) 
- [Risk Metrics](#Risk-Metrics) 
- [Accounting/Logging](#Accounting/Logging) 
- [CLI](#CLI) 

# API
## Gateway
### Functions provided
### Data provided
### Data required
### Modules with interaction

## Data Manager


## Agent Pool


## Portfolio Manager


## Backtesting

### S: request stock historical prices (DP)

parameters:
name|desc|type
-|-|-
stock_code| stock code|string
market_code|market code. stock may come from various market|string
start_time|timestamp|int
end_time|timestamp. these two time define the period of stock historical price I need |int
frequency|data frequency, e.g., "1s", "1min", "5min", "1day"|string

return:
name|desc|type
-|-|-
prices|historical prices|[Price](#price)[]

### S: request stock historical orderbooks (DP)

parameters:
name|desc|type
-|-|-
stock_code| stock code|string
market_code|market code. stock may come from various market|string
start_time|timestamp|int
end_time|timestamp. these two time define the period of stock historical price I need |int

return:
name|desc|type
-|-|-
orderbooks|historical orderbooks|[OrderBook](#orderbook)[]

### S: request transaction fees (DP)

parameters:
name|desc|type
-|-|-
market_code|market code. Fees might vary in various market|string

there are various fees: Commissions, Slippage, Spread, Data Fees, Clearing and Settlement Fees, Exchange Fees, Taxes, Liquidity Discounts or Rebates. The return should be a dictionary.

return:
name|desc|type
-|-|-
fees|fees|[dict](#dict)

e.g.

```json
{
  "Commissions": 10.0,
  "Exchange Fees": 11.0
}
```

### S: set agent (AP)

parameters:
name|desc|type
-|-|-
type|agent type, e.g. RSI|string
input_topic|the name of topic where agent should subscribe to get market data|string
output_topic|the name of topic where agent should publish its order|string

return:
name|desc|type
-|-|-
response||[Response](#response)

`response.msg` might be:
- Wrong agent type
- Memory Limit
- Unable to set agent for unknown reason

### S: start backtesting (UI)

parameters:
name|desc|type
-|-|-
type|backtesting type e.g. market replay|string
start_time|timestamp|int
end_time|timestamp|int
agent_type|e.g. RSI|string
param|additional parameters for backtesting or agent|[dict](#dict)

return:
name|desc|type
-|-|-
response||[Response](#response)

### S: handle order (AA2)

parameters:
name|desc|type
-|-|-
order|trading order|[Order](#order)

return:
name|desc|type
-|-|-
confirmation|It is always true. BM here acts as a fake CE. The order is always successful.|boolean

### T: market price (MDF)

name|desc|type
-|-|-
price|real-time data|[Price](#price)

### T: market orderbook (MDF)

name|desc|type
-|-|-
orderbook|real-time data|[OrderBook](#orderbook)

### T: simulated market price (AP)

name|desc|type
-|-|-
price|It might be the real-time data with noises; or historical data|[Price](#price)

### T: simulated market orderbook (AP)

name|desc|type
-|-|-
orderbook|It might be the real-time data with noises; or historical data|[OrderBook](#orderbook)

### Price

name|desc|type
-|-|-
stock_code|stock code|string
market_code|market code. stock may come from various market|string
open|open price|float
close|close price|float
high|high price|float
low|low price|float
time|timestamp|int
count|Total number of shares traded during the period|float
amount|Total value of shares traded during the period|float
frequency|data frequency, e.g., "1s", "1min", "5min", "1day"|string

### OrderBook

name|desc|type
-|-|-
orders|List of orders in the order book|Order[]

### Order

name|desc|type
-|-|-
stock_code|stock code|string
market_code|market code. stock may come from various market|string
price|price|float
size|size|float
side|buy(true) or sell(false)|boolean
time|timestamp|int

### dict

*We can always translate dictionary into ROS messages.*

https://github.com/Erio-Harrison/ros2_basic/blob/main/src/interfaces_demo/msg/Dictionary.msg

https://github.com/Erio-Harrison/ros2_basic/blob/main/src/interfaces_demo/msg/KeyValue.msg

### Response

name|desc|type
-|-|-
status|result of the operation, "success", "error" |string
msg|provide extra information|string

### 1. **Market Value of Open Positions**
- **Definition**: The current market value of all open (unclosed) positions. It represents the total value of assets currently held in the account.
- **Calculation Formula**:

$$ \text{Market Value of Open Positions} = \sum (\text{Quantity of each asset} \times \text{Current market price of the asset}) $$

### 2. **Total Return**
- **Definition**: The overall return on the portfolio over the entire period of the strategy, accounting for both realized and unrealized gains/losses.
- **Calculation Formula**:

$$\text{Total Return} = \frac{\text{Final portfolio value} - \text{Initial portfolio value}}{\text{Initial portfolio value}}$$

### 3. **Win Rate**
- **Definition**: The proportion of profitable trades out of the total number of trades executed.
- **Calculation Formula**:

$$\text{Win Rate} = \frac{\text{Number of profitable trades}}{\text{Total number of trades}}$$

### 4. **Profit-Loss Ratio**
- **Definition**: The ratio of the average profit from winning trades to the average loss from losing trades. It helps evaluate the risk-reward profile of a strategy.
- **Calculation Formula**:

$$\text{Profit-Loss Ratio} = \frac{\text{Average profit per winning trade}}{\text{Average loss per losing trade}}$$

### 5. **Average Trade Return**
- **Definition**: The average return per trade over the entire period. It provides insight into how much return each trade generates on average.
- **Calculation Formula**:

$$\text{Average Trade Return} = \frac{\sum (\text{Return of each trade})}{\text{Total number of trades}}$$

### 6. **Trade Frequency**
- **Definition**: The number of trades executed per unit of time (e.g., daily, weekly). It reflects the activity level of the trading strategy.
- **Calculation Formula**:

$$\text{Trade Frequency} = \frac{\text{Total number of trades}}{\text{Total time period (in days, weeks, etc.)}}$$

### 7. **Latency**
- **Definition**: The time delay between when a trading signal is generated and when the trade is executed. Lower latency is critical in high-frequency trading.
- **Calculation Formula**:

$$\text{Latency} = \text{Time of trade execution} - \text{Time of trade signal}$$

### 8. **Holding Time**
- **Definition**: The average duration for which positions are held before being closed. It is useful for understanding the time horizon of a trading strategy.
- **Calculation Formula**:

$$\text{Holding Time} = \frac{\sum (\text{Closing time of each position} - \text{Opening time of each position})}{\text{Total number of trades}}$$

### 9. **Sharpe Ratio**
- **Definition**: A measure of risk-adjusted return, it indicates how much excess return is earned per unit of risk taken by the strategy.
- **Calculation Formula**:

$$\text{Sharpe Ratio} = \frac{R_p - R_f}{\sigma_p}$$

Where:
- $R_p$ = Average return of the portfolio
- $R_f$ = Risk-free rate
- $\sigma_p$ = Standard deviation of the portfolio returns (representing risk)

**portfolio Return**:

$$\text{Portfolio Return} = \frac{V_t - V_{t-1}}{V_{t-1}}$$

Where:
- $V_t$ = Value of the portfolio at the end of the period
- $V_{t-1}$ = Value of the portfolio at the beginning of the period

## Action Approval

### Functions provided
#### Order Approval Process
- This function handles the entire process of approving or denying trading orders. It involves receiving orders, checking risk metrics, validating capital, and making a decision on whether to approve or reject the orders.
- Usage: 
  - The function interacts with the RiskMetrics module to assess the risk associated with each order.
  - It verifies capital requirements and checks the validity of the orders before making a decision.
- Function Prototypes:
  - bool approveOrder(Order order)
  - void approveOrders(Order orders[], int numOrders)
#### Order execution Confirmation
- After orders are approved and sent for execution, this function handles the confirmation of their execution status.
- Usage:
  - The function waits for a response from the ControlExternal module, which confirms whether the orders were successfully executed.
  - It then forwards the execution confirmation to the AgentPool and logs the details into the Accounting/Logging system.
- Function Prototypes:
  - ExecutionStatus confirmExecution(Order order)
  - void confirmExecutions(Order orders[], ExecutionStatus statuses[], int numOrders)

#### Order Denial Notification
- If an order fails validation or risk assessment, this function sends a denial notification to the 'AgentPool'.
- Usage:
  - The function handles the communication with the 'AgentPool' to inform the agent of the denial and the reasons
  - It will also logs the denial in the 'Accounting/Logging' module for future reference.
- Function Prototypes:
  - void notifyOrderDenial(Order order)
  - void notifyOrdersDenial(Order orders[], int numOrders)
### Data Provided
####
- Order package for risk metrics' assessment
- Approval status as `bool` indicating whether each order is approved or denied.
- Execution confirmation as `ExecutionStatus` indicating the result of the order execution.
- Denial notifications detailing why each order was denied.
#### Execution Confirmations:
- Status of the order execution, confirming if the approved orders were successfully executed.
#### Denial Notification:
- Information on why an order was denied, including 'Risk Metrics' and capital verification results
### Data Required
- Orders containing details like `double size`, `Enum position`, `String symbol`.
- Risk metrics such as Value at Risk (VaR), liquidity, and capital risk.
- Capital information, including available capital and constraints.
### Modules with interaction
- Agent Pool: Receives approval or denial notifications.
- Risk Metrics: Provides risk assessment data.
- Control External: Executes approved orders and provides execution confirmation.
- Accounting/Logging: Logs the results of the approval and execution process.

## Risk Metrics
### Functions provided
#### Value at Risk
- double var(double historicalData[], double size, Enum position)
- double var(Order order, Enum timeUnit, timestamp time, int steps)
  - Order contains: double size, Enum position, String symbol

#### Liquidity
- double liquidity(double orderBook[][])
- double liquidity(Order order, Enum timeUnit, timestamp time)

#### Capital Risk
- double correlation(double historicalData[][]) 
- double correlation(double historicalData1[], double historicalData1[])
- double correlation(Order order1, Order order2, Enum timeUnit, timestamp time, int steps)
- double waveletCoherence(double historicalData1[], double historicalData1[])
- double waveletCoherence(Order order1, Order order2, Enum timeUnit, timestamp time, int steps)

### Data provided
Risk metrics calculated in double.


### Data required
Historical price data, orderbook


### Modules with interaction
- Request Data Storage
- Requested by Action Approval
- Requested by Agent Pool


## Accounting/Logging


## CLI
