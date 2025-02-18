# Piper_Moveit  

[中文](README.md)  

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)  

|ROS |STATE|  
|---|---|  
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|  

## 1. Install Moveit Environment  
> Note: Moveit 1.1.11 is included in the `src` directory, no need to download separately.  

Source installation requires `wstool` and `catkin_tools`:  
```bash
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```  

## 2. Install Piper_ros  

Install dependencies:  
```bash
pip install numpy rospkg pyyaml  
pip3 install python-can  
pip3 install piper_sdk  
```  

Clone the source code and open a terminal:  
```bash
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-noetic-no-aloha
```  

Enter the workspace:  
```bash
cd Piper_ros
```  

Build the package:  
```bash
catkin_make
```  

## 3. Compile the Workspace  
> Compilation may fail due to conflicts between the Conda environment and the system environment. You can either uninstall Conda or use the system environment to compile.  

> For details, see [this blog](https://blog.csdn.net/endurance2017/article/details/102997980).  

Clone the source code and open a terminal:  
```bash
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git -b ros-noetic-moveit
```  

Enter the workspace:  
```bash
cd ~/Piper_ros_moveit
```  

Build the package:  
```bash
catkin_make
```  

## 4. Usage  
> Note: Every time you launch a new terminal, remember to `source` the setup script.  

### 1) Launch the Piper_ros Arm Control Node  

> Note: After restarting `moveit/demo.launch`, you need to restart the ROS control node. The control node can be started after MoveIt is running.  

Enter the workspace:  
```bash
cd Piper_ros  
source devel/setup.bash  
```  

Run the control node:  
```bash
roslaunch piper start_single_piper.launch
```  
> If the enabling process is successful, the node is ready.  

### 2) Run MoveIt  

Enter the workspace:  
```bash
cd ~/Piper_ros_moveit  
source devel/setup.bash  
```  

Run MoveIt (with gripper):  
```bash
roslaunch piper_with_gripper_moveit demo.launch
```  
> The gripper mode is divided into two control groups:  
> - **Arm Control Group**: Includes joints `joint1` to `joint6`.  
> - **Gripper Control Group**: Includes joints `joint7` and `joint8`, where `joint7` is actively controlled, and `joint8` is passively controlled.  

Run MoveIt (without gripper):  
```bash
roslaunch piper_no_gripper_moveit demo.launch
```  

To check joint state information, use:  
```bash
rostopic echo /joint_states
```  
> - The first six values correspond to arm position control.  
> - The seventh value corresponds to gripper position control.  
> - The eighth value is `0`, indicating it is not controlled.  

### 3) Plan and Execute Motion  

![](src/image/piper_moveit.png)  

After adjusting the position, click **"Plan & Execute"** under **MotionPlanning** on the left panel to start planning and execution.