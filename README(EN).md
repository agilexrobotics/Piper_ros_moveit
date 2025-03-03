# Piper_Moveit2

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> Note: For installation issues, refer to section 5.

## 1. Installing Moveit2

1) Binary Installation, [Reference Link](https://moveit.ai/install-moveit2/binary/)

    ```bash
    sudo apt install ros-humble-moveit*
    ```

2) Source Compilation, [Reference Link](https://moveit.ai/install-moveit2/source/)

## 2. Required Environment

After installing Moveit2, some dependencies need to be installed.

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

If the system locale is not set to English, set it as follows:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3. Workspace Compilation

Clone the source code, open the terminal:

```bash
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git
```

Navigate to the workspace:

```bash
cd Piper_ros_moveit
```

Compile:

```bash
colcon build
```

## 4. Moveit Control

### 4.1. Configuring Piper_ros

Configure the environment:

```bash
pip3 install python-can scipy piper_sdk catkin-pkg em
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

Source Compilation:

```bash
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-humble-no-aloha
cd ~/Piper_ros
colcon build 
```

Install dependencies:

```bash
sudo apt update && sudo apt install ethtool
sudo apt install can-utils
```

Activate the port:

```bash
cd ~/Piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

Start the control node:

```bash
ros2 launch piper start_single_piper.launch.py
```

### 4.2. Moveit2 Control

Start Moveit2:

```bash
cd ~/Piper_ros_moveit
conda deactivate # If you don't have a conda environment, remove this line
source install/setup.bash
```

For no gripper:

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

For gripper:

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

![piper_moveit](src/image/piper_moveit.png)

You can directly drag the end effector arrow to control the arm.

After adjusting the position, click the Plan & Execute button under MotionPlanning on the left to start planning and moving.

## 5. Moveit Control for Simulated Arm

### 5.1. Gazebo

#### 5.1.1. Running Gazebo

See [piper_sim](<https://github.com/agilexrobotics/piper_sim/tree/humble>) README 2.1

#### 5.1.2. Moveit Control

Same as [4.2 Moveit2 Control](#42-moveit2-control)

### 5.2. Mujoco

#### 5.2.1. Moveit Control (Run Moveit first)

Same as [4.2 Moveit2 Control](#42-moveit2-control)

#### 5.2.2. Running Mujoco

See [piper_sim](<https://github.com/agilexrobotics/piper_sim/tree/humble>) README 2.2

## 6. Possible Issues

### 6.1. Error when opening Gazebo: URDF not loaded, causing end effector to clip with the base in the simulation

1. Check if the `config` folder exists in `install` under `piper_description`, and ensure it contains files from `src/piper/piper_description/config`.

2. Similarly, check if `install` contains the necessary URDF files.

3. Ensure that the path in line 644 of `src/piper/piper_description/urdf/piper_description_gazebo.xacro` is correct. If the issue persists, change the path to an absolute path.

### 6.2. Error running demo.launch.py

Error: Parameter requires a double, but a string was provided.

Solution:
Run the following in the terminal:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

Or, set `LC_NUMERIC` before launching:

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```
