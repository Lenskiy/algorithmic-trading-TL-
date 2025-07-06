# Setting up Environment

---

## Prerequisites

Ensure you have the following installed:
- ROS2 iron
- Ubuntu 22.04 LTS
- C++17 Compiler

## Installation

### ROS2 Installation
Follow the instructions to install ROS2 iron on Ubuntu 20.04 LTS:
[ROS2 iron Installation Guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

### Configuring IntelliSense in VS Code

Ensure your have adequate path, then the configurations may look like this:

```bash
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "/opt/ros/iron/include",
                "${workspaceFolder}/**",
                "${workspaceFolder}/install/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64",
            "compilerArgs": []
        }
    ],
    "version": 4
}
```

### System Dependencies
Some additional dependencies may be required for system. Use the following command to install them:

1. CCAPI:<br>
Suggeted clone it on Desktop or you may modify the cmakelisy

```bash
git clone https://github.com/crypto-chassis/ccapi.git
```

Prepare the Build Directory:

```bash
cd /home/harrison/Desktop/ccapi/app
mkdir build
cd build
```

Run CMake and Make:

```bash
cmake ..
make
```
Notice, sometimes the ccapi may change their URL. If you meet problems for this library, please refer to:

[https://github.com/crypto-chassis/ccapi](https://github.com/crypto-chassis/ccapi)

Set the CCAPI_PATH environment variable:
Run the following command in a terminal (replace the path with your actual CCAPI installation path):

```bash
export CCAPI_PATH=/path/to/your/ccapi
```

To make this setting permanent, add this line to your `~/.bashrc` file:

```bash
echo 'export CCAPI_PATH=/path/to/your/ccapi' >> ~/.bashrc
source ~/.bashrc
```

2. Arrow:

There are various installation methods, which may be found in the official website of [Arrow Apache](https://arrow.apache.org/install/).

3. For boost, rapidjson, hffix

```sh
apt-get update
apt-get install -y build-essential cmake libboost-all-dev

mkdir -p /root/hffix
cd /root/hffix
wget https://github.com/jamesdbrock/hffix/archive/refs/tags/v1.3.0.tar.gz
tar -xzf v1.3.0.tar.gz --strip-components=1
rm v1.3.0.tar.gz

mkdir -p /root/rapidjson
cd /root/rapidjson
wget https://github.com/Tencent/rapidjson/archive/refs/tags/v1.1.0.tar.gz
tar -xzf v1.1.0.tar.gz --strip-components=1
rm v1.1.0.tar.gz
```
4.

To use Alpha Vantage as data source, we need to run and install:

```bash
sudo apt-get install nlohmann-json3-dev

sudo apt-get install libcurl4-openssl-dev

```

5.To use Polygon with websocket, we need to install:

```bash
sudo apt-get install libwebsocketpp-dev libssl-dev


## import .env

`export $(grep -v '^#' .env | xargs)`

## How to run

`1: `Before running, we need to compile. This command will compile all packages:

```bash
colcon build

```

## Compiling and running the environment

---
### 1 Compiling
Run
```bash
colcon build
```
to compile all the packages in the system.

If you updated and want compile a specific package, use:
```bash
colcon build --packages-select <package_name>
```
For example, if `action` relies on `interfaces`, you must have compiled `interfaces`.

```bash
# Example code
# Must replace package names with an existing package in the system. 
colcon build --pakcages-select interfaces
colcon build --pakcages-select action
```

### 2: Configure environment variables:

Open a terminal in the ros installation folder.
```bash
# Run the code to configure for ros2.
source install/setup.bash
```

You may also try:
```bash
source /opt/ros/iron/setup.bash
```

The configuration is only effective for current session of terminal. 
The next time before running the system, you may still need to configure the environment variables.

To save time everytime a new terminal opened, you may try to edit `.bashrc` file:

```bash
nano ~/.bashrc
```

Add the following lines at the end of the file and save:

```bash
source /path_to_ros2_iron/setup.bash
source /path_to_project/install/local_setup.bash
```

This will take effect once a new terminal is opened. Or run the following command to apply the changes.
```bash
source ~/.bashrc
```

### 3: Running the system

```bash
# Example code
ros2 run <package_name> <executable_name>
```

Like:
```bash
ros2 run data_pulling data_pull
```

___
`Note: ` Some packages may depend on `interfaces` and sometimes we might need to manually add the installation path to CMAKE_PREFIX_PATH:
```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:Desktop/trading_TL_2024/install
```

If still not work, please `Debugging Further`:
```bash
colcon build --packages-select action --event-handlers console_direct+

