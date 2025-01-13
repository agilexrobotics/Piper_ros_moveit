import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import launch_ros
import launch

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

# 这个文件是用来启动Gazebo仿真环境的，其中包含了两个节点，一个是Gazebo仿真环境，一个是Piper的模型
def generate_launch_description():
    # 获取 piper_description 包的路径
    pkg_share = FindPackageShare(package='piper_description').find('piper_description')
    urdf_file = os.path.join(pkg_share, 'urdf/piper_description_gazebo.xacro')

    # 设置环境变量 GAZEBO_MODEL_PATH
    set_env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', pkg_share)

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.0',
                              description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0',
                              description='Yaw angle of starting orientation'),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='Name and prefix for this robot'),

        # 设置环境变量
        set_env_var,

        # robot_state_publisher 发布 robot_description 参数
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # parameters=[{
            #     'use_sim_time': True,
            #     'robot_description': Command(
            #         [f'xacro {urdf_file}', ' robot_name:=', LaunchConfiguration('robot_name')])
            parameters=[{
                        'use_sim_time': True,
                        'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command([
                        'xacro ',os.path.join(pkg_share,urdf_file)]), value_type=str)  }]
            ),
        
        # Node(
            # package='controller_manager',
            # executable='spawner.py',
            # arguments=['arm_controller', '--stopped'],  # 控制器初始化为停止状态
            # output='screen'
            # ),


        # Gazebo spawn entity 将机器人加载到 Gazebo 中
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('robot_name'),
                '-topic', 'robot_description',  # 使用 robot_description 参数
                '-x', LaunchConfiguration('start_x'),
                '-y', LaunchConfiguration('start_y'),
                '-z', LaunchConfiguration('start_z'),
                '-Y', LaunchConfiguration('start_yaw'),
                '-timeout', '1000'
            ],
            output='screen')
        
    ])
