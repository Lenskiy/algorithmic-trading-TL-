# 更新包管理器并安装必要的软件包
apt-get update
apt-get install -y build-essential cmake libboost-all-dev

# 清理apt缓存
rm -rf /var/lib/apt/lists/*
