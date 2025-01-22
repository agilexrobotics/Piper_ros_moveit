import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_moveit_config").to_moveit_configs()
    
    # 获取当前目录
    current_directory = os.path.dirname(os.path.realpath(__file__))
    
    # 添加执行 ordered_joint_states.py 脚本的动作
    ordered_joint_states_script = ExecuteProcess(
        cmd=["python3", "ordered_joint_states.py"],
        cwd=current_directory,  # 设置为当前目录
        output="screen"
    )
    
    return LaunchDescription([
        generate_demo_launch(moveit_config),
        ordered_joint_states_script
    ])
