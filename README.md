# Piper_Moveit

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1、安装Moveit环境
> 注：moveit 1.1.11包含在src中，无需单独下载

源安装需要 wstool，catkin_tools：
```
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```

## 2、安装Piper_ros
安装依赖
```
pip install numpy rospkg pyyaml
pip3 install python-can
pip3 install piper_sdk
```
git源码，打开终端
```
git clone https://github.com/agilexrobotics/Piper_ros.git -b ros-noetic-no-aloha
```
进入工作空间

```
cd Piper_ros
```
编译
```
catkin_make
```

## 3、工作空间编译
>此处编译可能因为conda环境和系统环境冲突，可以卸载conda解决或者使用系统环境编译

>问题详情可见 https://blog.csdn.net/endurance2017/article/details/102997980

git源码，打开终端
```
git clone https://github.com/agilexrobotics/Piper_ros_moveit.git -b ros-noetic-moveit
```

进入工作空间

```
cd ~/Piper_ros_moveit
```
编译
```
catkin_make
```

## 4、使用方法
> 注：在新终端运行launch是都需source一次

1）使用Piper_ros打开机械臂控制节点

进入工作空间

```
cd Piper_ros
source devel/setup.bash
```
运行节点
```
roslaunch piper start_single_piper.launch
```
>出现使能成功即可

2）运行moveit
进入工作空间

```
cd ~/Piper_ros_moveit
source devel/setup.bash
```
运行(有夹爪)
```
roslaunch piper_with_gripper_moveit demo.launch
```
3）规划轨迹并运动

![](src/image/piper_moveit.png)

调整好位置后点击左侧MotionPlanning中Planning的Plan&Execute即可开始规划并运动