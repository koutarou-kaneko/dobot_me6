# This is for DOBOT magician E6.
Especially for operation with Ubuntu 22.04 + ROS-O.

This repository is developed from https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis

## Setup

First, install ros-o.

```bash
echo "export DOBOT_TYPE=me6" >> ~/.bashrc
cd ${YOUR_WS}/src # Please replace ${YOUR_WS} with path of your workspace
git clone https://github.com/koutarou-kaneko/moveit.git
git clone https://github.com/koutarou-kaneko/ruckig.git
git clone https://github.com/PickNikRobotics/rosparam_shortcuts.git
git clone https://github.com/ompl/ompl.git
cd /usr/local/lib
sudo ln -s libompl.so.19 libompl.so.17
sudo ldconfig
catkin build
```

## How to use
```bash
roslaunch me6_bringup bringup.launch
```
There are 3 args.

###### When you use the real machine
Set the "real_machine" to true.

###### When you use the Gazebo
Set the "simulation" to true.

###### When you use the Moveit!
Set the "moveit" to true.

