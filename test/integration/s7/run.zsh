SESSION_NAME="TEST_collect_data"

tmux new-session -d -s $SESSION_NAME
tmux split-window -v

# Pane 0
tmux send-keys -t 0 "source install/setup.zsh" C-m 
tmux send-keys -t 0 "ros2 run data_pulling data_pull" C-m 

# Pane 1
tmux send-keys -t 1 "source install/setup.zsh" C-m  
tmux send-keys -t 1 "ros2 run data_storage write_data" C-m  

tmux attach-session -t 0.1
