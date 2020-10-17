#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import time

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from probot_msgs.msg import SetOutputIO
from object_color_detector.srv import *

redStore   = [0.3154, -0.2254]
greenStore = [0.3154, -0.3808]
blueStore  = [0.1391, -0.2254]

capture_point = Point(0.308, 0, 0.435)
capture_quaternion = Quaternion(0, 0, -0.2505, 0.9681) # Quaternion(0, 0, 0, 1)

pick_red_height   = 0.18
pick_green_height = 0.157
pick_blue_height  = 0.18
pick_prepare_height = 0.3

place_prepare_height = 0.4

red_count   = 0
green_count = 0
blue_count  = 0

class ProbotSortingDemo:
    def __init__(self):
        # Initialize ros and moveit 
        moveit_commander.roscpp_initialize(sys.argv)
                
        # Initialize moveit commander
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # Initialize arm effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # Initialize reference frame
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # Initialize moveit parameters
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)

        # Initialize IO
        self.ioPub = rospy.Publisher('probot_set_output_io', SetOutputIO, queue_size=1)

        #initialize arm position to home
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

    def moveToHome(self):
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)   

    def moveTo(self, x, y, z): 
        # Create pose data           
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation = capture_quaternion
        
        # Set pick position 
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        traj = self.arm.plan()

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def pick(self, x, y, z):
        if self.moveTo(x, y, pick_prepare_height) == True:
            print "Pick Once"
            self.moveTo(x, y, z)

            ioOutput = SetOutputIO()
            ioOutput.type = SetOutputIO.TYPE_RELAY
            ioOutput.mask = 3
            ioOutput.status = 3
            self.ioPub.publish(ioOutput)
            
            rospy.sleep(1)
            self.moveTo(x, y, pick_prepare_height)
            
            return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y, z): 
        if self.moveTo(x, y, place_prepare_height) == True:
            print "Place Once"
            self.moveTo(x, y, z)
            
            ioOutput = SetOutputIO()
            ioOutput.type = SetOutputIO.TYPE_RELAY
            ioOutput.mask = 3
            ioOutput.status = 0
            self.ioPub.publish(ioOutput)

            rospy.sleep(1)
            self.moveTo(x, y, place_prepare_height)

        else:
            print "Can not place"

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":

    time.sleep(3)

    rospy.init_node('probot_vision_sorting_demo')
    rate = rospy.Rate(10)

    reg_x = rospy.get_param('/image/reg_x')
    reg_y = rospy.get_param('/image/reg_y')

    print "Probot sorting demo start."
    demo = ProbotSortingDemo()
    
    while not rospy.is_shutdown():
        # 相机拍照位置
        demo.moveTo(capture_point.x, capture_point.y, capture_point.z)

        # Get target
        rospy.wait_for_service('/object_detect')
        try:
            detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
            response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if response.result is not DetectObjectSrvResponse.SUCCESS:
            rospy.loginfo("No objects detected, waiting detecting...")
            rate.sleep()
            continue
        
        rospy.loginfo("Get object position, Start pick and place.")

        # Pick and place bject
        if len(response.redObjList):
            x_value = response.redObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.redObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_red_height) == True:
                demo.place(redStore[0]-red_count*0.03, redStore[1]-red_count*0.03, pick_red_height+0.01)
                red_count = red_count + 1
        elif len(response.greenObjList):
            x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_green_height) == True:
                demo.place(greenStore[0]-green_count*0.03, greenStore[1]-green_count*0.03, pick_green_height+0.01)
                green_count = green_count + 1
        elif len(response.blueObjList):
            x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
            y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
            print "Pick Position: %f, %f"%(x_value, y_value)
            if demo.pick(x_value,  y_value, pick_blue_height) == True:
                demo.place(blueStore[0]-blue_count*0.03, blueStore[1]-blue_count*0.03, pick_blue_height+0.01)
                blue_count = blue_count + 1

        rate.sleep()

    demo.moveToHome()   
    demo.shutdown()
