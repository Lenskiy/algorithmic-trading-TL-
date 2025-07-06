SESSION_NAME="TEST_real_market"

tmux new-session -d -s $SESSION_NAME
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -h -t 0
tmux split-window -h -t 2
tmux split-window -h -t 4

# Pane 0
tmux send-keys -t 0 "source install/setup.zsh" C-m 
tmux send-keys -t 0 "ros2 run agent_pool ag1 --ros-args -p input_topic:=agent_in -p output_topic:=incoming_orders" C-m 

# Pane 1
tmux send-keys -t 1 "source install/setup.zsh" C-m  
tmux send-keys -t 1 "sleep 3 && echo 'Executing after 3 seconds'" C-m
tmux send-keys -t 1 "ros2 run backtesting bt2 --ros-args -p feed:=agent_in -p feedback:=approved_orders -p duration:=60.0" C-m

# Pane 2
tmux send-keys -t 2 "source install/setup.zsh" C-m  
tmux send-keys -t 2 "ros2 run action_approval action_approval" C-m  

# Pane 3
tmux send-keys -t 3 "source install/setup.zsh" C-m  
tmux send-keys -t 3 "ros2 run risk_metrics risk_server" C-m  

# Pane 4
tmux send-keys -t 4 "source install/setup.zsh" C-m  
tmux send-keys -t 4 "ros2 run data_storage read_data" C-m  

# Pane 5
tmux send-keys -t 5 "source install/setup.zsh" C-m  
tmux send-keys -t 5 "ros2 run data_storage write_data" C-m  

# Pane 6
tmux send-keys -t 6 "source install/setup.zsh" C-m  
tmux send-keys -t 6 "ros2 run data_pulling data_pull" C-m  

tmux attach-session -t 0.6
