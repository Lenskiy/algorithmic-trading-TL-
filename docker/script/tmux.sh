apt update
apt install -y tmux

echo 'set -g status-left-length 100   # 调整左侧状态栏长度为 100 字符' > ~/.tmux.conf
echo 'set -g status-right-length 100  # 调整右侧状态栏长度为 100 字符' >> ~/.tmux.conf
