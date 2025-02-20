#!/usr/bin/env python

import rospy
import time
import random
from moveit_demo.srv import JointMoveitCtrl, JointMoveitCtrlRequest

def call_joint_moveit_ctrl_arm(joint_states, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_arm")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_arm", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = joint_states
        request.gripper = 0.0
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_arm")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_arm, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def call_joint_moveit_ctrl_gripper(gripper_position, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_gripper")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_gripper", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = [0.0] * 6
        request.gripper = gripper_position
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_gripper")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_gripper, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def randomval():
    arm_position = [
        random.uniform(-0.2, 0.2),  # 关节1
        random.uniform(0, 0.5),  # 关节2
        random.uniform(-0.5, 0),  # 关节3
        random.uniform(-0.2, 0.2),  # 关节4
        random.uniform(-0.2, 0.2),  # 关节5
        random.uniform(-0.2, 0.2)   # 关节6
    ]
    
    gripper_position = random.uniform(0, 0.035)
    return arm_position, gripper_position

def main():
    rospy.init_node("test_joint_moveit_ctrl", anonymous=True)
    arm_position, gripper_position = [], 0
    for i in range(10): 
        arm_position, gripper_position = randomval()
        call_joint_moveit_ctrl_gripper(gripper_position)
        call_joint_moveit_ctrl_arm(arm_position)

if __name__ == "__main__":
    main()
