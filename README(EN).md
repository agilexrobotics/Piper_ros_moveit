# Piper_Moveit2

[中文](README(CN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> Note: If you encounter issues during installation and usage, refer to Section 5.

## 1. Install Moveit2

(1) Binary Installation, [Reference Link](https://moveit.ai/install-moveit2/binary/)

```bash
sudo apt install ros-humble-moveit*
```

(2) Source Compilation Method, [Reference Link](https://moveit.ai/install-moveit2/source/)

## 2. Environment Setup

After installing Moveit2, some dependencies need to be installed:

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

If your system's language region setting is not English, you need to set it:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3. Workspace Compilation

Clone the source code:

```bash
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git
```

Enter the workspace:

```bash
cd Piper_ros_moveit
```

Compile:

```bash
colcon build
```

## 4. Moveit Control

### 4.1 Moveit Control for Gazebo Simulated Robot Arm

> Note: Before using this, ensure that the processes started in Section 4.1 are closed. Running them simultaneously is not supported.

1) Start the Gazebo Simulation:

```bash
ros2 launch piper_description piper_gazebo.launch.py
```

Follow Section 4.1 to add the model. If the simulated robot arm is already present in the display window, no additional action is required.

![](src/image/piper_gazebo.png)

2) Use Moveit2 to Control the Robot Arm:

> Note: If Moveit2 fails to control the Gazebo model, restart the simulation.

```bash
ros2 launch piper_moveit_config demo.launch.py
```

![](src/image/piper_gazebo_moveit.png)

Once the position is adjusted, click "Plan & Execute" under the MotionPlanning panel to start the motion planning.

At this point, the robot model in Gazebo should begin moving.

### 4.3 Moveit Control for a Real Robot Arm

(1) Configure `piper_ros`

Set up the environment:

```bash
pip3 install python-can scipy piper_sdk catkin-pkg em
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

Compile the source code:

```bash
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-humble-no-aloha
cd ~/Piper_ros
colcon build 
```

(2) Control the Robot Arm

Install dependencies:

```bash
sudo apt update && sudo apt install ethtool
sudo apt install can-utils
```

Activate the CAN port:

```bash
cd ~/Piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

Start the control node:

```bash
ros2 launch piper start_single_piper.launch.py
```

Start Moveit2:

```bash
cd ~/Piper_ros_moveit
conda deactivate # Remove this line if you are not using Conda
source install/setup.bash
```

Run without a gripper:

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

Run with a gripper:

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

![](src/image/piper_moveit.png)

You can directly drag the end-effector arrows to control the robot arm.

After adjusting the position, click "Plan & Execute" under the MotionPlanning panel to start planning and executing the movement.

## 5. Possible Issues

### 5.1 Error when launching Gazebo: URDF not loaded, causing end-effector to pass through the base

(1) Check whether the `install` directory contains `piper_description/config`, and whether it includes the necessary files from `src/piper/piper_description/config`.

The same applies if the `install` directory is missing `urdf`.

(2) Check if the path in `src/piper/piper_description/urdf/piper_description_gazebo.xacro` (line 644) is correct. If the issue persists, try using an absolute path.

### 5.3 Error when running `demo.launch.py`

**Error:** A parameter requires a `double`, but a `string` was provided.

**Solution:**
Run the following command in the terminal:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

Alternatively, set `LC_NUMERIC` before running the launch file:

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```
