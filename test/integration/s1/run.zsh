SESSION_NAME="demo"

# 创建一个新的 tmux 会话并立即分离（-d 后台创建）
tmux new-session -d -s $SESSION_NAME
tmux split-window -h
tmux split-window -h 

# 运行命令到不同面板
tmux send-keys -t 0 "echo 1" C-m  # 向面板 0 发送命令
tmux send-keys -t 1 "echo 2" C-m  # 向面板 1 发送命令
tmux send-keys -t 2 "echo 3" C-m  # 向面板 2 发送命令

# 附加到会话以查看结果
tmux attach-session -t 0.2
