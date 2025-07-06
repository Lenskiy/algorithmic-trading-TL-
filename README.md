# Algorithmic-Trading
This respository is to develop an automatic trading system under ROS structure with live data access, financial order submission, data storage, financial strategies and backtesting modules. This project is conducted as part of the ANU course COMP4500/COMP8715 TechLauncher with guidance of Dr Artem Lenskiy.

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

## Contributing to the project
This is for better team management, that everyone should read carefully and follow the workflow when contributing to the repo.

### Create new branch and merge
Rather than directly working on the main branch, please work on the new branches you created.
- Create new branch from main.
- Implement something.
- Checkout to main, pull others' updates to main branch.
- Merge main onto the new branch.
- Push the new branch and create a merge request.
- After reviewing, squash and merge

### Commit message
Please have a read of the article [_How to Write a Git Commit Message_](https://cbea.ms/git-commit/).
Have a look on the examples of good commit messages. 

## Project management
Administrative tasks and documentations are managed in team's google drive.

[Team management google drive](https://drive.google.com/drive/folders/1VDCyogdQlhUGbavJ641idX5T8tAB1Pk8)
- [Statement of Work](https://drive.google.com/file/d/1n9YOyQfvwEV51zGj2XOcSB4R9e1RQSbn/view?usp=drive_link)
- [Team Charter](https://drive.google.com/file/d/1ScFmd9IKYy0MGs3lxVoDOpghl7bmRunN/view?usp=drive_link)
- [Decision logs](https://docs.google.com/spreadsheets/d/1Kb3knyPm5wvWdcbDP9MgwDxH3Jth2pkD-bY5KZmei04/edit?gid=0#gid=0)
- [Reflection logs](https://docs.google.com/spreadsheets/d/1ptWoBxOX69pi-OBlQIXWuk9xTYUcP42cfzIZ_Y97XJI/edit?gid=0#gid=0) 
- [Client meeting minutes](https://drive.google.com/drive/folders/1Sfe_-VWAU61cn0kfBl2O7VqQ3Fv8oKB-)
- [Team meeting minutes](https://drive.google.com/drive/folders/10jxrGva7KIV7pwVHxQ09_JfGLqpnyqr9)

## Team members
| First Name | Last Name | Preferred Name<br>(if any) | uid      | Git username        | Role                       |
|------------|-----------|----------------------------|----------|---------------------|----------------------------|
| Jinming    | Cao       |                            | u7439146 | Xmpoi0610           | ROS and backend developer  | 
| Haoren     | Hu        | Harrison                   | u7541840 | Erio-Harrison       | ROS and backend developer  |
| Xinyang    | Li        |                            | u7760022 | bridgeL             | Trading Strategy developer |
| Dafu       | Lyu       |                            | u7004308 | DafuLyu             | Database developer         |
| Han        | Xu        | Susan                      | u7590767 | SusanXu0110         | Trading Strategy developer |
| Ke         | Xu        | Alan                       | u7520531 | u7520531@anu.edu.au | ROS and backend developer  |
| Huanjie    | Zhang     | Jay                        | u7633338 | Mcnagi              | Spokesperson               |

