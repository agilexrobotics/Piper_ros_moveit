# Piper_Moveit

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 Install Moveit Environment

> Note: Moveit 1.1.11 is included in the src, no need to download separately.

Source installation requires wstool, catkin_tools:

```bash
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```

## 2 Install Piper_ros

Install dependencies:

```bash
pip install numpy rospkg pyyaml
pip3 install python-can
pip3 install piper_sdk
```

Clone the source from git, open the terminal:

```bash
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-noetic-no-aloha
```

Navigate to the workspace:

```bash
cd Piper_ros
```

Compile:

```bash
catkin_make
```

## 3 Workspace Compilation

> Compilation may conflict due to the conda environment and system environment. You can uninstall conda or use the system environment for compilation. For more details, refer to [solution](https://blog.csdn.net/endurance2017/article/details/102997980).

Clone the source from git, open the terminal:

```bash
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git -b ros-noetic-moveit
```

Navigate to the workspace:

```bash
cd ~/Piper_ros_moveit
```

Compile:

```bash
catkin_make
```

## 4 Usage

### 4.1 Start Piper_ros to Open Robotic Arm Control Node

> Note: After restarting moveit/demo.launch, the ROS control node needs to be restarted. The node can be started after running moveit.

Navigate to the workspace:

```bash
cd Piper_ros
source devel/setup.bash
```

Run the node (gripper control value doubled):

```bash
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

> If the enable message appears successfully, it's ready to go.

### 4.2 Run Moveit

Navigate to the workspace:

```bash
cd ~/Piper_ros_moveit
source devel/setup.bash
```

#### 4.2.1 Run (With Gripper)

```bash
roslaunch piper_with_gripper_moveit demo.launch
```

To run without RViz:

```bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

> The gripper mode is divided into two control groups:
>
>- **Arm Control Group** includes joints 1 to 6.
>- **Gripper Control Group** includes joints 7 and 8, with joint 7 for active control and joint 8 for passive control.
>- **Piper Control Group** includes joints 1 and 6, joint 7 for gripper control.
> The gripper control range is from 0 to 0.035 meters, corresponding to an actual gripper opening/closing distance of 0 to 0.07 meters.

|joint_name|     limit(rad)     |    limit(angle)    |     limit(rad/s)   |   limit(rad/s^2)   |
|----------|     ----------     |     ----------     |     ----------     |     ----------     |
|joint1    |   [-2.618, 2.618]  |    [-150.0, 150.0] |      [0, 3.0]      |      [0, 5.0]      |
|joint2    |   [0, 3.14]        |    [0, 180.0]      |      [0, 3.0]      |      [0, 5.0]      |
|joint3    |   [-2.967, 0]      |    [-170, 0]       |      [0, 3.0]      |      [0, 5.0]      |
|joint4    |   [-1.745, 1.745]  |    [-100.0, 100.0] |      [0, 3.0]      |      [0, 5.0]      |
|joint5    |   [-1.22, 1.22]    |    [-70.0, 70.0]   |      [0, 3.0]      |      [0, 5.0]      |
|joint6    |   [-2.0944, 2.0944]|    [-120.0, 120.0] |      [0, 3.0]      |      [0, 5.0]      |

Control information node is `/joint_states`:

```bash
rostopic echo /joint_states
```

>- The first 6 values are for arm position control.
>- The 7th value is for gripper position control.
>- The 8th value is 0, not involved in control.

#### 4.2.2 Run (Without Gripper)

```bash
roslaunch piper_no_gripper_moveit demo.launch
```

To run without RViz:

```bash
roslaunch piper_no_gripper_moveit demo.launch use_rviz:=false
```

## 4.3 Plan Trajectories and Move

### 4.3.1 Teach Mode via Dragging

![piper_moveit](src/image/piper_moveit.png)

After adjusting the position, click "Plan & Execute" under "MotionPlanning" on the left side to start planning and moving.

### 4.3.2 Server Control (Joint Angle Control)

Control the robotic arm (enter in terminal):

```bash
cd Piper_ros_moveit
source devel/setup.bash
```

Arm joint angle control:

```bash
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
max_velocity: 0.5
max_acceleration: 0.5" 
```

End pose control for the robotic arm:

```bash
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 
```

Gripper control:

```bash
rosservice call /joint_moveit_ctrl_gripper "gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

Control both arm and gripper together:

```bash
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

### 4.3.3 Client Control (Enter in terminal)

```bash
cd Piper_ros_moveit
source devel/setup.bash
rosrun moveit_demo joint_moveit_ctrl.py
```

> Modify the arm_position and gripper_position in [joint_moveit_ctrl](src/moveit_demo/scripts/joint_moveit_ctrl.py) to control joint movement, units in radians.

### 4.3.4 Moveit Class Control (Joint Angle Control)

> This section can be added in the [joint_moveit_ctrl](src/moveit_demo/scripts/joint_moveit_ctrl.py) to apply.

```python
#!/usr/bin/env python3

import rospy
import moveit_commander

def move_robot():
    # Initialize MoveIt! components
    moveit_commander.roscpp_initialize([])
    move_group = moveit_commander.MoveGroupCommander("gripper")  # Modify for "arm", "gripper" or "piper" as needed

    # Get current joint values
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0  # For arm use joint_goal length of 6, for gripper use length of 1, and for piper use length of 7
    # joint_goal[1] = 0.0
    # joint_goal[2] = 0.0
    # joint_goal[3] = 0.0
    # joint_goal[4] = 0.0
    # joint_goal[5] = 0.0
    # joint_goal[6] = 0.0  # Piper planning group uses this for gripper control

    # Set and execute the target
    move_group.set_joint_value_target(joint_goal)
    success = move_group.go(wait=True)
    rospy.loginfo(f"Movement success: {success}")
    rospy.loginfo(f"joint_value: {move_group.get_current_joint_values()}")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
```

## 5 MoveIt Control for Simulated Robotic Arm

### 5.1 Gazebo

#### 5.1.1 Gazebo Configuration

See [piper_sim](<https://github.com/agilexrobotics/piper_sim>) README section 2.1.

#### 5.1.2 MoveIt Control  

Same as [4.2 Run MoveIt](#42-run-moveit).

### 5.2 Mujoco  

#### 5.2.1 Running Mujoco

See [piper_sim](<https://github.com/agilexrobotics/piper_sim>) README section 2.2.  

#### 5.2.2 MoveIt Control

Same as [4.2 Run MoveIt](#42-run-moveit)
