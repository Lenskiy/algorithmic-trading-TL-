## How to run the agent pool node:
```bash
ros2 run agent_pool ap
```

## How to start an agent:
```bash
ros2 service call /set_agent "system_interface/srv/SetAgent" "{agent_type: 'RSI', input_topic: 'market_data', output_topic: 'action_approval_input'}"
```
This command starts an agent with the strategy `RSI`, which receives price data from the topic `market_data` and sends orders to `action_approval_input`. The service returns an `agent_id` and a `bool` value (indicating whether the agent was successfully started).

## How to close an agent:
```bash
ros2 service call /close_agent "system_interface/srv/CloseAgent" "{agent_id: '4'}"
```

## How to list all agents:
```bash
ros2 service call /list_agents "std_srvs/srv/Empty"
```
