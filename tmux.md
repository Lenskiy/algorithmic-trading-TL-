如何使用tmux管理终端

# 如何退出TMUX

`tmux kill-session`

# Why

为何使用tmux？

- 窗口管理：tmux 提供了强大的窗口和面板管理功能，可以轻松拆分窗口为多个面板，且易于在不同面板之间切换。
- 脚本支持：tmux 的配置文件和命令脚本非常灵活，支持自动化操作。你可以通过脚本轻松自动化创建多个窗口和执行不同命令。
- 状态栏：tmux 有一个默认的状态栏，可以显示当前会话、窗口信息等，可以通过配置自定义它的显示内容。

对于我们的交易系统，需要在多个进程下启动这些节点，并查看他们的运行情况，使用tmux进行集成测试是个不错的选择

# How

`tmux` 的三级结构：会话 / 窗口 / 面板

### 会话相关指令：
- 新建会话：`tmux new-session -s <session_name>`
- 新建后台会话：`tmux new-session -d -s <session_name>`
- 列出所有会话：`tmux ls`
- 关闭所有会话：`tmux kill-server`
- 关闭指定会话：`tmux kill-session -t <session_name>`
- 恢复一个后台会话：`tmux attach-session -t <session_name>`
- 将会话转入后台：`tmux detach`

### 窗口相关指令：
- 创建窗口：`tmux new-window -n <window_name>`
- 切换窗口：`tmux select-window -t <window_number>`
- 查看所有窗口：`tmux list-windows`
- 删除窗口：`tmux kill-window -t <window_number>`
- 删除当前窗口：`tmux kill-window`

### 面板相关指令：
- 创建面板（水平分割）：`tmux split-window -h`
- 创建面板（垂直分割）：`tmux split-window -v`
- 切换面板：`tmux select-pane -t <pane_number>` 或 `Ctrl + b` 然后按方向键
- 调整面板大小：`Ctrl + b` 然后按住 `Ctrl` 并使用箭头键调整
- 列出所有面板：`tmux list-panes`
- 关闭面板：`tmux kill-pane -t <pane_number>`
- 关闭当前面板：`tmux kill-pane`
- 交换面板位置：`tmux swap-pane -s <pane_number_1> -t <pane_number_2>`

- 发送命令到指定面板并执行：`tmux send-keys -t <pane_number> "echo hi" C-m`

C-m: 这是 tmux 中表示 "回车" 的命令，表示发送命令后立即执行

### 快捷键

切换不同的面板：先按`ctrl+B`，然后松手，再按`方向键`


