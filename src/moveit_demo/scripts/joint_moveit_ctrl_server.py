#!/usr/bin/env python

import rospy
from moveit_commander import *
from moveit_demo.srv import JointMoveitCtrl, JointMoveitCtrlResponse

class JointMoveitCtrlServer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('joint_moveit_ctrl_server')

        # 初始化 MoveIt
        roscpp_initialize([])
        self.robot = RobotCommander()

        # 获取 MoveIt 规划组列表
        available_groups = self.robot.get_group_names()
        rospy.loginfo(f"Available MoveIt groups: {available_groups}")

        # 仅实例化存在的规划组
        self.arm_move_group = None
        self.gripper_move_group = None
        
        if "arm" in available_groups:
            self.arm_move_group = MoveGroupCommander("arm")
            rospy.loginfo("Initialized arm move group.")
        
        if "gripper" in available_groups:
            self.gripper_move_group = MoveGroupCommander("gripper")
            rospy.loginfo("Initialized gripper move group.")

        # 创建服务
        self.arm_srv = rospy.Service('joint_moveit_ctrl_arm', JointMoveitCtrl, self.handle_joint_moveit_ctrl_arm)
        self.gripper_srv = rospy.Service('joint_moveit_ctrl_gripper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_gripper)
        rospy.loginfo("Joint MoveIt Control Services Ready.")

    def handle_joint_moveit_ctrl_arm(self, request):
        rospy.loginfo("Received arm joint movement request.")

        try:
            if self.arm_move_group:
                arm_joint_goal = request.joint_states[:6]  # 取前六个关节作为机械臂目标
                self.arm_move_group.set_joint_value_target(arm_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)  # 等待机械臂完成运动
                rospy.loginfo("Arm movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during arm movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_gripper(self, request):
        rospy.loginfo("Received gripper joint movement request.")

        try:
            if self.gripper_move_group:
                gripper_goal = [request.gripper]  # 假设夹爪只有 1 个关节
                self.gripper_move_group.set_joint_value_target(gripper_goal)
                self.gripper_move_group.go(wait=True)  # 等待夹爪完成运动
                rospy.loginfo("Gripper movement executed successfully.")
            else:
                rospy.logerr("Gripper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during gripper movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

if __name__ == '__main__':
    JointMoveitCtrlServer()
    rospy.spin()
