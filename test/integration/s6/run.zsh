SESSION_NAME="TEST_order_risk"

tmux new-session -d -s $SESSION_NAME
tmux split-window -v 
tmux split-window -v

# Pane 0
tmux send-keys -t 0 "source install/setup.zsh" C-m 
tmux send-keys -t 0 "ros2 run data_storage read_data" C-m 

# Pane 1
tmux send-keys -t 1 "source install/setup.zsh" C-m  
tmux send-keys -t 1 "ros2 run risk_metrics risk_server" C-m  

# Pane 2
tmux send-keys -t 2 "source install/setup.zsh" C-m  
tmux send-keys -t 2 "ros2 service call /order_risk \"system_interface/srv/OrderRisk\" \"{symbol: 'bitusd', exchange: 'bitmex', price: 100, size: 100, side: true, time: 0}\"" C-m  

tmux attach-session -t 0.2
