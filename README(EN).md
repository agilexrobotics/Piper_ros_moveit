# Piper_Moveit

[中文](README.MD)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1. Install Moveit Environment
> Note: Moveit 1.1.11 is included in the src folder, no need to download it separately.

Source installation requires wstool and catkin_tools:
```
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```

## 2. Install Piper_ros
Install dependencies
```
pip install numpy rospkg pyyaml
pip3 install python-can
pip3 install piper_sdk
```

Clone the source code, open a terminal
```
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-noetic-no-aloha
```
Navigate to the workspace
```
cd Piper_ros
```
Build
```
catkin_make
```
## 3. Build the Workspace
> The build may fail due to conflicts between the conda and system environments. You can uninstall conda or use the system environment to compile.

> For more details on the issue, refer to: https://blog.csdn.net/endurance2017/article/details/102997980

Clone the source code, open a terminal
```
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git -b ros-noetic-moveit
```
Navigate to the workspace
```
cd ~/Piper_ros_moveit
```
Build
```
catkin_make
```
## 4. Usage
> Note: Remember to source the environment in a new terminal before running the launch files.

1) Open the robotic arm control node with Piper_ros

Navigate to the workspace
```
cd Piper_ros
source devel/setup.bash
```
Run the node
```
roslaunch piper start_single_piper.launch
```
> The "Enabled" message will appear if it is successful.

2) Run Moveit
Navigate to the workspace
```
cd ~/Piper_ros_moveit
source devel/setup.bash
```
Run
```
roslaunch piper_moveit demo.launch
```
3) Plan the trajectory and move

![](src/image/piper_moveit.png)

Once the position is adjusted, click "Plan & Execute" under "MotionPlanning" on the left to begin planning and executing the movement.
