# Piper_Moveit2

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> 注：安装使用过程中出现问题可查看第5部分

## 1、安装Moveit2

1）二进制安装，[参考链接](https://moveit.ai/install-moveit2/binary/)

```bash
sudo apt install ros-humble-moveit*
```

2）源码编译方法，[参考链接](https://moveit.ai/install-moveit2/source/)

## 2、使用环境

安装完Moveit2之后，需要安装一些依赖

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

若系统语言区域设置不为英文区域，须设置

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3、工作空间编译

git源码，打开终端

```bash
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git
```

进入工作空间

```bash
cd Piper_ros_moveit
```

编译

```bash
colcon build
```

## 4、moveit控制

### 4.1、配置piper_ros

配置环境

```bash
pip3 install python-can scipy piper_sdk catkin-pkg em
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

源码编译

```bash
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-humble-no-aloha
cd ~/Piper_ros
colcon build 
```

安装依赖

```bash
sudo apt update && sudo apt install ethtool
sudo apt install can-utils
```

激活端口

```bash
cd ~/Piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

开启控制节点

```bash
ros2 launch piper start_single_piper.launch.py
```

### 4.2、moveit2控制

开启moveit2

```bash
cd ~/Piper_ros_moveit
conda deactivate # 若无conda环境可去除此行
source install/setup.bash
```

无夹爪运行

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

有夹爪运行

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

![piper_moveit](src/image/piper_moveit.png)

可以直接拖动机械臂末端的箭头控制机械臂

调整好位置后点击左侧MotionPlanning中Planning的Plan&Execute即可开始规划并运动

## 5、moveit控制仿真机械臂

### 5.1、gazebo

#### 5.1.1、运行gazebo

见 [piper_sim](<https://github.com/agilexrobotics/piper_sim/tree/humble>)的 README 2.1

#### 5.1.2 moveit控制

同 [4.2 moveit2控制](#42moveit2控制)

### 5.2、mujoco

#### 5.2.1、moveit控制（先运行moveit）

同 [4.2 moveit2控制](#42moveit2控制)

#### 5.2.2、运行mujoco

见 [piper_sim](<https://github.com/agilexrobotics/piper_sim/tree/humble>)的 README 2.2

## 6、可能遇见的问题

### 6.1、打开gazebo时报错，提示urdf未加载，导致仿真环境中机械臂末端与底座穿模

1、注意编译后的install下piper_description中是否有config，且config中是否包含src/piper/piper_description中config的文件

install中缺少urdf同理

2、注意src/piper/piper_description/urdf/piper_description_gazebo.xacro中644行的路径是否正确，如确认后问题依然存在，将路径改为绝对路径

### 6.2、运行demo.launch.py时报错

报错：参数需要一个double，而提供的是一个string
解决办法：
终端运行

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

或在运行launch前加上LC_NUMERIC=en_US.UTF-8
例如

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```
