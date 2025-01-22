# Piper_Moveit2

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)


|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> Note: If issues occur during installation or usage, refer to Section 5.

## 1. Install Moveit2

1) Binary installation, [reference link](https://moveit.ai/install-moveit2/binary/)

```
sudo apt install ros-humble-moveit*
```

2) Source compilation method, [reference link](https://moveit.ai/install-moveit2/source/)


## 2. Environment Setup

After installing Moveit2, you need to install some dependencies

```
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```
If your system locale is not set to an English region, set it to:

```
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```
## 3. Workspace Compilation
Clone the source code, open a terminal
```
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git
```

Navigate to the workspace

```
cd Piper_ros_moveit
```
Build
```
colcon build
```
## 4. Usage
> Note: Remember to source the environment in a new terminal before running launch files.

Navigate to the workspace
```
cd ~/Piper_ros_moveit
```
source
```
source install/setup.bash
```
### 4.1. Moveit RViz Simulation

1) View the Piper robotic arm model

```
ros2 launch piper_description display_piper.launch.py 
```

After opening successfully, you need to add the model in Rviz. If the robotic arm is already displayed, no further addition is necessary.

In the lower-left corner of the display, add "RobotModel" and set the Description Topic to /robot_description.

Settings on the left as shown below:

![](src/image/piper.png)

You can directly control the robotic arm joints through the pop-up control window.

2) Use Moveit2 to control the robotic arm

(You can use the previous step independently. After running, you can directly add the model. If the robotic arm is already displayed, no need to add it again.)

```
ros2 launch piper_moveit_config demo.launch.py
```

![](src/image/piper_moveit.png)

You can directly drag the arrow on the end of the robotic arm to control it.

After adjusting the position, click "Plan & Execute" or "Plan" under "MotionPlanning" on the left to begin planning.

### 4.2. Start Gazebo Simulation
> Note: You must close the processes opened in 4.1 before using this, as both cannot be opened simultaneously.

1) Start Gazebo simulation

```
ros2 launch piper_description piper_gazebo.launch.py
```

Follow the same steps as in 4.1 to add the model. If the robotic arm is already displayed, no need to add it again.

![](src/image/piper_gazebo.png)

2) Use Moveit2 to control the robotic arm

> Note: Sometimes Moveit may not be able to control the Gazebo model. If this happens, you need to restart.


```
ros2 launch piper_moveit_config demo.launch.py
```


![](src/image/piper_gazebo_moveit.png)

After adjusting the position, click "Plan & Execute" under "MotionPlanning" to begin planning.

At this point, you can see the model in Gazebo starting to move.

### 4.3. Moveit2 Control of Real Robotic Arm Piper

1) Configure Piper_ros

Set up the environment

```
pip3 install python-can scipy piper_sdk catkin-pkg em
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

Source compilation

```
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-humble-no-aloha
cd ~/Piper_ros
colcon build 
```
2) Control the robotic arm

Install dependencies
```
sudo apt update && sudo apt install ethtool
sudo apt install can-utils
```
Activate the port
```
cd ~/Piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```
Start the control node

```
ros2 launch piper start_single_piper.launch.py
```

Start Moveit2

```
cd ~/Piper_ros_moveit
conda deactivate # 若无conda环境可去除此行
source install/setup.bash
ros2 launch piper_moveit_config demo.launch.py
```


![](src/image/piper_moveit.png)

You can directly drag the arrow on the end of the robotic arm to control it.

After adjusting the position, click "Plan & Execute" under "MotionPlanning" to begin planning and movement.

## 5. Potential Issues

### 5.1. Compilation Errors

    Install the missing packages according to the error messages. If using conda, pay attention to the environment path issues.

### 5.2. Gazebo Launch Error: URDF Not Loaded, Arm End Effector and Base Model Intersect in Simulation

1. Ensure that there is a config folder in `piper_description` in the install directory, and check if it contains the config files from `src/piper/piper_description/config`.

2. Similarly, check if the URDF is missing in the install directory.

3. Ensure that the path in `src/piper/piper_description/urdf/piper_description_gazebo.xacro` at line 644 is correct. If the issue persists, try using the absolute path.

### 5.3. Error Running demo.launch.py

Error: A double parameter is required, but a string is provided.

Solution:
Run the following in the terminal

```
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

Or add `LC_NUMERIC=en_US.UTF-8` before running the launch file:

```
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```