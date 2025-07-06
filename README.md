## Contents
- [Introduction](#introduction)
- [System design](#system-design)
- [Running the program](#running-the-program)
- [Contributing to the project](#contributing-to-the-project)
- [Project management](#project-management)
- [Team members](#team-members)

## Introduction
There are many trading bots but none of them use Robotic Operating System (ROS). In this project, ROS helps to connect 
different modules concerned with rational trading in the goal of small latency and large scalalbility.

This project aims to receive live data and submit orders to exchanges and interactive brokers. Data is stored locally in
parquet files. After data is communicated throughout the system by ROS messages, the trading decision is created by trading strategies module, tested by backtesting module, and assessed by risk management module. 


## System design
The system composer describes the interrelationship among the modules.
![system composer.png](admin/system%20composer.png)
<br>
Sequence diagram depicts more dynamic view of the communication and actions. This is ordered.
The sequence diagram also captures some parallel messages and conditioned actions, as in `PAR` and `OPT` operands.
![sequence_diagram.png](admin/sequence%20diagram.png)

## Running the program
This project is implemented under ROS framework. To get started, you may first visit the ros2-iron website to download 
the environment. <br>
Part of the dependencies in this project only support Linux Operating system. 
To have a working environment, you may also install docker ([docker installation](docker/readme.md)) on other operating systems.<br>

To get started, please check out the following documentations:<br>
- [Setting up environment](admin/documents/Setting%20up%20environment.md)
- [Backtesting module](src/backtesting/readme.md)
- [Agent Pool module](agent_pool/readme.md)
