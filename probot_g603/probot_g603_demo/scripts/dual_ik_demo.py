#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from probot_msgs.msg import ControllerCtrl  

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('dual_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        L_arm = moveit_commander.MoveGroupCommander('L_manipulator')
        R_arm = moveit_commander.MoveGroupCommander('L_manipulator')
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        L_end_effector_link = L_arm.get_end_effector_link()
        R_end_effector_link = R_arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        L_reference_frame = 'ground'
        R_reference_frame = 'ground'
        L_arm.set_pose_reference_frame(L_reference_frame)
        R_arm.set_pose_reference_frame(R_reference_frame)
                
        # 当运动规划失败后，允许重新规划
        L_arm.allow_replanning(True)
        R_arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        L_arm.set_goal_position_tolerance(0.001)
        L_arm.set_goal_orientation_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        R_arm.set_goal_position_tolerance(0.001)
        R_arm.set_goal_orientation_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
       
        # 设置允许的最大速度和加速度
        L_arm.set_max_acceleration_scaling_factor(0.5)
        L_arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_acceleration_scaling_factor(0.5)
        R_arm.set_max_acceleration_scaling_factor(0.5)
        R_arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        L_target_pose = PoseStamped()
        L_target_pose.header.frame_id = L_reference_frame
        L_target_pose.header.stamp = rospy.Time.now()     
        L_target_pose.pose.position.x = 0.35
        L_target_pose.pose.position.y = 0.1
        L_target_pose.pose.position.z = 0.273714
        L_target_pose.pose.orientation.w = 1.0

        R_target_pose = PoseStamped()
        R_target_pose.header.frame_id = R_reference_frame
        R_target_pose.header.stamp = rospy.Time.now()     
        R_target_pose.pose.position.x =-0.25
        R_target_pose.pose.position.y = 0.1
        R_target_pose.pose.position.z = 0.273714
        R_target_pose.pose.orientation.w = 1.0
        
        # 设置机器臂当前的状态作为运动初始状态
        L_arm.set_start_state_to_current_state()
        R_arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        L_arm.set_pose_target(L_target_pose, L_end_effector_link)
        R_arm.set_pose_target(R_target_pose, R_end_effector_link)
        
        # 规划运动路径
        L_traj = L_arm.plan()
        R_traj = R_arm.plan()
        L_points = len(L_traj.joint_trajectory.points) 
        R_points = len(R_traj.joint_trajectory.points) 

        joint_positions = []
        L_joint_positions = list(L_traj.joint_trajectory.points[L_points - 1].positions)
        R_joint_positions = list(R_traj.joint_trajectory.points[R_points - 1].positions)
        joint_positions = L_joint_positions
        joint_positions.extend(R_joint_positions)

        arm.set_joint_value_target(joint_positions)       
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
