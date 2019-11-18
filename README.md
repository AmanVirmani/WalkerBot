# Turtlebot Walker
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Project Overview
This project implements a roomba type behavior robot using turtlebot platform in ROS. This repository contains the following files: 
- include/walker/walker.h
- src/walker.cpp
- src/main.cpp
- launch/demo.launch

# Dependencies 
- ROS Kinetic 
To install follow this [link](http://wiki.ros.org/kinetic/Installation)
- Ubuntu 16.04
- Turtlebot packages 
To install turtlebot, type the following:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

# Building package 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/AmanVirmani/WalkerBot
cd ..
catkin_make
```

# Running demo 
## Using roslaunch 
Type the following command in a new terminal:
```
roslaunch walkerbot demo.launch
```

## Using rosrun 
Run roscore in a new terminal 
```
roscore
```
launch turtlebot gazebo simulation 
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Run the turtlebot node by using the following command 
```
rosrun walkerbot walkerbot 
```

# Recording using rosbag files 
Record the rostopics using the following command with the launch file (camera output is not recorded): 
```
roslaunch walkerbot demo.launch record:=true
```
The recorded bag file will be stored in the results folder 

To record for a specific time 
```
roslaunch walkerbot demo.launch record:=true secs:=30
```
In the above case, rosbag will record for 30 seconds

# Playing bag files 
Navigate to the results folder 
```
cd ~/catkin_ws/src/turtlebot_walker/results 
```
Play the bag file 
```
rosbag play turtlebotRecord.bag
```
Verify the published topic by given command 
```
rostopic echo /mobile_base/commands/velocity
```

