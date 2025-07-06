# 联调：agent_pool, backtesting
SESSION_NAME="TEST_data_read"

tmux new-session -d -s $SESSION_NAME
tmux split-window -v
tmux split-window -h -t 0
tmux split-window -h -t 2

# Pane 0
tmux send-keys -t 0 "source install/setup.zsh" C-m 
tmux send-keys -t 0 "ros2 run agent_pool ap" C-m 

# Pane 1
tmux send-keys -t 1 "source install/setup.zsh" C-m  
tmux send-keys -t 1 "ros2 run backtesting btm" C-m  

# Pane 2
tmux send-keys -t 2 "source install/setup.zsh" C-m  
tmux send-keys -t 2 "ros2 run data_storage read_data" C-m  

# Pane 3
tmux send-keys -t 3 "source install/setup.zsh" C-m  
tmux send-keys -t 3 "sleep 3 && echo 'Executing after 3 seconds'" C-m
tmux send-keys -t 3 "ros2 service call /run_backtesting \"system_interface/srv/RunBacktesting\" \"{bt_type: 'bt3', bt_params_json: '{}'}\"" C-m  

tmux attach-session -t 0.3
