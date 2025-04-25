# EW458-Final-Project-2025: Create3 Robot LIDAR Mapping
![image](https://github.com/user-attachments/assets/8fc3ff95-41d3-46e3-bd70-330c68290362)


## Install Dependencies

Clone repository:
```
https://github.com/odoy25/EW458-Final-Project.git
```
roslibpy:
```
pip install roslibpy 
```
deque:
```
pip install deque
```
Visual Occupancy Map (Connect to Common Local Network):
```
http://192.168.8.104:8080/map.html
```
## Overview

Our Firstie Team designed an integrated ROS2 python script that subscribes and publishes to a create3 robot in order to produce an ocupancy map. 

Internal Odometry and LIDAR scan messages are subscribed from the create3 and sent over a LAN network to a computer node. Visual Studio Code proccess these messages, calculates, creates and then publishes a ROS Occupancy message to the robot. 

The map can be viewed online through the robot's IP address and map topic while the node is connected to the same network.

The map displays a 2D image with obstacles (black), free-space (white), and unknown (red). A grey dot (cross) is displayed to represent the robot's current locaiton. 

![image](Hopper_Hall_LIDAR_scan.png)

## What is an Occupancy Map?
In order for an unmanned system to navigate through an environment, it must have an understanding of what obstacles or other features might lie in its path. Typically this information is displayed on a map. Figuring out where that system lies on a map without outside help like GPS is a process known as **localization**. The standard map used for localization with robotics and their navigation is called an **Occupancy Grid**.

An Occupancy Grid divides a space into small squares (in 2D) or cubes (in 3D), each referred to as a cell. Each cell is assigned a numerical value to represent a feature of the environment, in this case whether it is free space (0), occupied by an obstacle (100), or unknown (-1). The system’s odometry provides an estimate of its position, which is used to build the map over time. LiDAR measurements are also incorporated, which provide data on the location of obstacles relative to the robot’s frame of reference. For each LiDAR measurement, the obstacle’s location is calculated in the odometry reference frame, and the nearest cell is assigned a value if it achieves a high probability of fulfulling one of the listed values. 

Atthe beginning of the trial, all cells are unknown (-1). As the vehicle continues to move and gather more sensor data, the occupancy grid is continually updated until the environment is entirely mapped, now incorporting free or occupied space,  allowing for future navigation and path planning.

## TG30 LIDAR 


## How It Works



## How Code Works

![image](HKO_map1.gif)
