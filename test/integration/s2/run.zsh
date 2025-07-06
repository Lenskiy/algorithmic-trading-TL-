SESSION_NAME="TEST_csv_backtesting"

tmux new-session -d -s $SESSION_NAME
tmux split-window -h

# Pane 0
tmux send-keys -t 0 "source install/setup.zsh" C-m 
tmux send-keys -t 0 "ros2 run agent_pool ag2 --ros-args -p input_topic:=agent_in -p output_topic:=agent_out -p mix:=false" C-m 

# Pane 1
tmux send-keys -t 1 "source install/setup.zsh" C-m  
tmux send-keys -t 1 "ros2 run backtesting bt3 --ros-args -p feed:=agent_in -p feedback:=agent_out" C-m  

tmux attach-session -t 0.1
