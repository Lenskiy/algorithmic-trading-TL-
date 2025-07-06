# 创建目标目录并进入
mkdir -p /root/hffix
cd /root/hffix

# 下载并解压hffix库
wget https://github.com/jamesdbrock/hffix/archive/refs/tags/v1.3.0.tar.gz
tar -xzf v1.3.0.tar.gz --strip-components=1

# 删除压缩包
rm v1.3.0.tar.gz
