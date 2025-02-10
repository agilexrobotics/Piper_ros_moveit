import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 设置 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit_config").to_moveit_configs()
      
    return LaunchDescription([
        generate_demo_launch(moveit_config),  # 加载 MoveIt 配置
    ])
