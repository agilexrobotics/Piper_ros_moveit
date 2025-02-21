# Piper_Moveit

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 安装Moveit环境

> 注:moveit 1.1.11包含在src中,无需单独下载

源安装需要 wstool,catkin_tools:

```bash
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```

## 2 安装Piper_ros

安装依赖

```bash
pip install numpy rospkg pyyaml
pip3 install python-can
pip3 install piper_sdk
```

git源码,打开终端

```bash
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-noetic-no-aloha
```

进入工作空间

```bash
cd Piper_ros
```

编译

```bash
catkin_make
```

## 3 工作空间编译

>此处编译可能因为conda环境和系统环境冲突,可以卸载conda解决或者使用系统环境编译, 问题详情可见 https://blog.csdn.net/endurance2017/article/details/102997980

git源码,打开终端

```bash
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git -b ros-noetic-moveit
```

进入工作空间

```bash
cd ~/Piper_ros_moveit
```

编译

```bash
catkin_make
```

## 4 使用方法

### 4.1 使用Piper_ros打开机械臂控制节点

>注:每次重启moveit/demo.launch后需要重启ros控制节点,节点可在运行moveit后启动

进入工作空间

```bash
cd Piper_ros
source devel/setup.bash
```

运行节点(夹爪控制值二倍)

```bash
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

>出现使能成功即可

### 4.2 运行moveit

进入工作空间

```bash
cd ~/Piper_ros_moveit
source devel/setup.bash
```

### 4.2.1 运行(有夹爪)

```bash
roslaunch piper_with_gripper_moveit demo.launch
```

若不启动rviz运行

```bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

>夹爪模式分为两个控制组:
>- **机械臂控制组** 包含关节 joint1 至 joint6
>- **夹爪控制组** 包含关节 joint7 和 joint8, 夹爪控制组采用 joint7 进行主动控制,而 joint8 为被动控制
>- **piper控制组** 包含关节 joint1 和 joint6, joint7为夹爪控制
> 夹爪控制值范围为0到0.035, 单位为m, 对应到实际夹爪张合距离需乘2,即0到0.07

|joint_name|     limit     |
|----------|  ---------    |
|joint1    | [-2.618,2.618]|
|joint2    | [0,3.14]|
|joint3    | [-2.697,0]|
|joint4    | [-1.832,1.832]|
|joint5    | [-1.22,1.22]|
|joint6    | [-3.14,3.14]|
|joint7    | [0,0.035]|
|joint8    | [-0.035,0]|

控制信息节点为/joint_states

```bash
rostopic echo /joint_states
```

>- 其中前6个值为机械臂位置控制
>- 第7个值为夹爪位置控制
>- 第8个值为0不参与控制

### 4.2.2 运行(无夹爪)

```bash
roslaunch piper_no_gripper_moveit demo.launch
```

若不启动rviz运行

```bash
roslaunch piper_no_gripper_moveit demo.launch use_rviz:=false
```

## 4.3 规划轨迹并运动

### 4.3.1 拖动示教

![](src/image/piper_moveit.png)

调整好位置后点击左侧MotionPlanning中Planning的Plan&Execute即可开始规划并运动

### 4.3.2 服务端控制(关节弧度控制)

控制机械臂 (终端输入)

```bash
cd Piper_ros_moveit
source devel/setup.bash
```

机械臂关节弧度控制

```bash
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
max_velocity: 0.5
max_acceleration: 0.5" 

```

机械臂末端位置控制

```bash
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 

```

控制夹爪 (终端输入)

```bash
rosservice call /joint_moveit_ctrl_gripper "gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

控制机械臂和夹爪联合运动

```bash
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

### 4.3.3 客户端控制 (终端输入)

```bash
cd Piper_ros_moveit
source devel/setup.bash
rosrun moveit_demo joint_moveit_ctrl.py
```

> 更改 [joint_moveit_ctrl](src/moveit_demo/scripts/joint_moveit_ctrl.py)中的 arm_position, gripper_position 控制关节运动,单位为弧度

### 4.3.4 moveit类控制 (关节弧度控制)

> 此部分可加在 [joint_moveit_ctrl](src/moveit_demo/scripts/joint_moveit_ctrl.py) 中应用

```python
#!/usr/bin/env python3

import rospy
import moveit_commander

def move_robot():
    # 初始化 MoveIt! 相关组件
    moveit_commander.roscpp_initialize([])
    move_group = moveit_commander.MoveGroupCommander("gripper")  # 可以根据需要修改为 "arm"、"gripper"或"piper"

    # 获取当前关节值
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0  # arm使用joint_goal长度为6,gripper使用joint_goal长度为1,piper使用joint_goal长度为7
    # joint_goal[1] = 0.0
    # joint_goal[2] = 0.0
    # joint_goal[3] = 0.0
    # joint_goal[4] = 0.0
    # joint_goal[5] = 0.0
    # joint_goal[6] = 0.0 # 使用piper规划组是，这一位是夹爪控制

    # 设置并执行目标
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
