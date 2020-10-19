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
        self.L_arm = moveit_commander.MoveGroupCommander('L_manipulator')
        self.R_arm = moveit_commander.MoveGroupCommander('R_manipulator')
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        self.L_end_effector_link = self.L_arm.get_end_effector_link()
        self.R_end_effector_link = self.R_arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        self.L_reference_frame = 'L_base_link'
        self.R_reference_frame = 'R_base_link'
        self.L_arm.set_pose_reference_frame(self.L_reference_frame)
        self.R_arm.set_pose_reference_frame(self.R_reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.L_arm.allow_replanning(True)
        self.R_arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.L_arm.set_goal_position_tolerance(0.001)
        self.L_arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.R_arm.set_goal_position_tolerance(0.001)
        self.R_arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
       
        # 设置允许的最大速度和加速度
        self.L_arm.set_max_acceleration_scaling_factor(0.5)
        self.L_arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.R_arm.set_max_acceleration_scaling_factor(0.5)
        self.R_arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
               
        joint_dest = self.getTraj(0.25,-0.1,0.2,0.25,0.1,0.2)

        self.arm.set_joint_value_target(joint_dest)       
        # 控制机械臂完成运动
        self.arm.go()

        joint_dest = self.getTraj(-0.25,-0.1,0.2,0.25,0.1,0.2)

        self.arm.set_joint_value_target(joint_dest)       
        # 控制机械臂完成运动
        self.arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def getTraj(self,x1, y1, z1, x2, y2, z2):
        L_target_pose = PoseStamped()
        L_target_pose.header.frame_id = self.L_reference_frame
        L_target_pose.header.stamp = rospy.Time.now()     
        L_target_pose.pose.position.x = x1
        L_target_pose.pose.position.y = y1
        L_target_pose.pose.position.z = z1
        L_target_pose.pose.orientation.w = 1.0

        R_target_pose = PoseStamped()
        R_target_pose.header.frame_id = self.R_reference_frame
        R_target_pose.header.stamp = rospy.Time.now()     
        R_target_pose.pose.position.x = x2
        R_target_pose.pose.position.y = y2
        R_target_pose.pose.position.z = z2
        R_target_pose.pose.orientation.w = 1.0

        # 设置机器臂当前的状态作为运动初始状态
        self.L_arm.set_start_state_to_current_state()
        self.R_arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.L_arm.set_pose_target(L_target_pose, self.L_end_effector_link)
        self.R_arm.set_pose_target(R_target_pose, self.R_end_effector_link)
        
        # 规划运动路径
        L_traj = self.L_arm.plan()
        R_traj = self.R_arm.plan()
        L_points = len(L_traj.joint_trajectory.points) 
        R_points = len(R_traj.joint_trajectory.points) 

        joint_positions = []
        L_joint_positions = list(L_traj.joint_trajectory.points[L_points - 1].positions)
        R_joint_positions = list(R_traj.joint_trajectory.points[R_points - 1].positions)
        joint_positions = L_joint_positions
        joint_positions.extend(R_joint_positions)
        
        return joint_positions

if __name__ == "__main__":
    MoveItIkDemo()

    
    
