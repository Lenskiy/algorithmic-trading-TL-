# 创建目标目录
mkdir -p /root/rapidjson
cd /root/rapidjson

# 下载并解压 rapidjson
wget https://github.com/Tencent/rapidjson/archive/refs/tags/v1.1.0.tar.gz
tar -xzf v1.1.0.tar.gz --strip-components=1

# 删除压缩包
rm v1.1.0.tar.gz
