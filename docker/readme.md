# How to run ROS2 in docker

## Install 

### install docker-desktop

download it here [https://www.docker.com/products/docker-desktop/](https://www.docker.com/products/docker-desktop/)

### install docker-image

1. open your docker-desktop 
2. open a new terminal at the root path of this repository 
3. run `docker-compose up` at the terminal

It will take you around **30 minutes** for installing everything you need.

This image includes:

```
curl
wget
git
zsh
arrow
ccapi
ros2
colcon
python3.10
```

## Open Docker Container Exec

### For vscode user

1. install vscode extension: `Dev Containers`
2. `Ctrl+Shift+P` Open the command panel and run `dev containers: attach to running container`
3. the project is under `/root/trading`

this will help your vscode connected to the ubuntu container in docker

### For pure user

`docker exec -it my_container /bin/bash`

## Set up your git in Docker Container

```sh
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
```

## Test

```sh
cd $PRJ_ROOT
source $ROS2_SETUP
colcon build --packages-select system_interface
colcon build --packages-select data_pulling
source install/setup.zsh
ros2 run data_pulling data_pull
```

## How to Update .env

Changes of `.env` won't be applied immediately. 

Please run `export $(grep -v '^#' .env | xargs)` at the root path of this project

## How to speed up vscode in docker

How to speed up vscode in docker, since it is a cpu and memory limited environment?

### Change Vscode Configuration file:

Add following configuration

```json
"files.watcherExclude": {
    "**/build/**": true,
    "**/log/**": true,
    "**/install/**": true,
    "**/__pycache__/**": true,
    "**/*.pyc": true
}
```

### Create pyrightconfig.json file at the root path of project

```json
{
  "exclude": [
    "**/node_modules",
    "**/__pycache__",
    "**/.git",
    "build",
    "install",
    "log"
  ]
}
```

so that you can enhance the performance of your python language server

