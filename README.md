# This is for DOBOT magician E6.
Especially for operation with Ubuntu 22.04 + ROS-O.

This repository is developed from https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis

## Setup

First, install ros-o.

```bash
cd ${YOUR_WS}/src # Please replace ${YOUR_WS} with path of your workspace
echo "export DOBOT_TYPE=me6" >> ~/.bashrc
git clone https://github.com/koutarou-kaneko/dobot_me6
git clone https://github.com/koutarou-kaneko/moveit.git
git clone https://github.com/koutarou-kaneko/ruckig.git
sudo apt install libfmt-dev
```
```bash
git clone https://github.com/PickNikRobotics/rosparam_shortcuts.git
```
Modify rosparam_shortcuts/CMakeLists.txt as follows:
```bash
- add_compile_options(-std=c++14)
+ add_compile_options(-std=c++17)
```
```bash
git clone https://github.com/koutarou-kaneko/ompl.git
cd ompl
mkdir -p build/Release
cd build/Release
cmake ../..
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~/usr/local/lib
sudo ln -s libompl.so.19 libompl.so.17
sudo ldconfig
```
```bash
cd ${YOUR_WS}/src
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

