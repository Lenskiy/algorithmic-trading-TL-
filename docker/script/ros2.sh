# install ROS2
apt install software-properties-common -y
add-apt-repository universe -y
apt update 
apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt upgrade -y
apt install ros-iron-desktop -y

# Set environment variables and installation environment for ROS 2 source
echo "source /opt/ros/iron/setup.zsh" >> ~/.zshrc

# install Python 3.10 and pip
apt install -y python3.10 python3.10-venv python3.10-dev 
curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

# install colcon
apt install -y python3-colcon-common-extensions
