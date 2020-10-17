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
import numpy as np
import math
import yaml

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from object_color_detector.srv import *
from sklearn.linear_model import LinearRegression

# 定义标定位姿点
calibration_points_z = 0.2 #0.07
calibration_points = []
calibration_points.append(Point(0.30, 0,    calibration_points_z))
calibration_points.append(Point(0.30, 0.1,  calibration_points_z))
calibration_points.append(Point(0.40, 0.1, calibration_points_z))
calibration_points.append(Point(0.40, -0.1,  calibration_points_z))
calibration_points.append(Point(0.40, 0,    calibration_points_z))

# 定义拍照位姿
capture_point = Point(0.308, 0, 0.435)
capture_quaternion = Quaternion(0, 0, -0.2505, 0.9681) #Quaternion(0, 0, 0, 1)

# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)

# 初始化ROS节点
rospy.init_node('auto_calibration')
        
# 初始化需要使用move group控制的机械臂中的arm group
arm = moveit_commander.MoveGroupCommander('manipulator')
        
# 获取终端link的名称
end_effector_link = arm.get_end_effector_link()
print(end_effector_link)
                
# 设置目标位置所使用的参考坐标系
reference_frame = 'base_link'
arm.set_pose_reference_frame(reference_frame)
        
# 当运动规划失败后，允许重新规划
arm.allow_replanning(True)

# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
arm.set_goal_position_tolerance(0.0001)
arm.set_goal_orientation_tolerance(0.0001)

# 控制机械臂先回到初始化位置
arm.set_named_target('home')
arm.go()
rospy.sleep(2)

# 设置机械臂工作空间中的目标位姿
target_pose = PoseStamped()
target_pose.header.frame_id = reference_frame
target_pose.header.stamp = rospy.Time.now()     
target_pose.pose.orientation = capture_quaternion

capture_pose = target_pose
capture_pose.pose.position = capture_point

# 设置图像识别为调试模式
config_filename = rospy.get_param("~filename")

# 标定参数
regObjList  = []

image_params = rospy.get_param("/image")
success_count= 0

xarray = np.zeros(len(calibration_points))
yarray = np.zeros(len(calibration_points))
xc_array = np.zeros(len(calibration_points))
yc_array = np.zeros(len(calibration_points))


def moveTo(x, y, z): 
    # Create pose data           
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    target_pose.pose.orientation = capture_quaternion
    
    # Set pick position 
    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, end_effector_link)
    
    traj = arm.plan()
    arm.execute(traj)

# 进入标定循环
for point in calibration_points:
    # 设置标定位姿
    target_pose.pose.position = point

    # 运动到标定位姿
    arm.set_pose_target(target_pose, end_effector_link)
    arm.go()
    rospy.sleep(1)
    
    # 等待用户确认
    input_value = raw_input("请将蓝色标定物放置到吸盘正下方，按任意键确认！")
    rospy.sleep(1)

    # 运动到拍照位姿
    moveTo(capture_point.x, capture_point.y, capture_point.z)
    rospy.sleep(1)

    # 目标识别
    rospy.loginfo("尝试识别蓝色标定物...")
    try:
        # 请求服务
        rospy.wait_for_service('/object_detect')
        try:
            detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
            response = detect_object_service(DetectObjectSrvRequest.RED_OBJECT) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        regObjList = response.blueObjList
        rospy.loginfo("Detect red object over" )
    except rospy.ROSException:
        rospy.loginfo("Timeout waiting for image data.")

    if len(regObjList) == 1:        
        # 像素坐标
		xc_array[success_count] = regObjList[0].position.y
		yc_array[success_count] = regObjList[0].position.x
        # 空间坐标
		xarray[success_count] = point.x
		yarray[success_count] = point.y

		rospy.loginfo("%d: 空间坐标: %f, %f, 像素坐标: %d, %d" % (success_count, xarray[success_count], yarray[success_count], xc_array[success_count], yc_array[success_count]))

		xc_array = xc_array.reshape(-1,1)
		yc_array = yc_array.reshape(-1,1)
		xarray = xarray.reshape(-1,1)
		yarray = yarray.reshape(-1,1)

		success_count = success_count + 1

		rospy.loginfo("第%d个点标定成功！", success_count)
    else:
        rospy.loginfo("第%d个点标定失败，尝试标定下一个点！", success_count)

#计算标定结果
if success_count < 3:
    rospy.loginfo("标定点数量不够（%d<3），标定失败！", success_count)
else:
    Reg_x_xc = LinearRegression().fit(xc_array, xarray)
    Reg_y_yc = LinearRegression().fit(yc_array, yarray)
    k1 = Reg_x_xc.coef_[0][0]
    b1 = Reg_x_xc.intercept_[0]
    k2 = Reg_y_yc.coef_[0][0]
    b2 = Reg_y_yc.intercept_[0]
    rospy.loginfo("Linear Regression for x and xc is :  x = %.5fxc + (%.5f)" % (k1, b1))
    rospy.loginfo("Linear Regression for y and yc is :  y = %.5fyc + (%.5f)" % (k2, b2))

    with open(config_filename, "r") as f:
        content = yaml.load(f.read())
        # 修改yml文件中的参数
        content['image']['reg_x'][0] = float(k1)
        content['image']['reg_x'][1] = float(b1)
        content['image']['reg_y'][0] = float(k2)
        content['image']['reg_y'][1] = float(b2)

    with open(config_filename, "w") as nf:
        yaml.dump(content, nf)

    rospy.set_param("/image/reg_x", content['image']['reg_x']);
    rospy.set_param("/image/reg_y", content['image']['reg_y']);

# 控制机械臂回到初始化位置
arm.set_named_target('home')
arm.go()

# 关闭并退出moveit
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

