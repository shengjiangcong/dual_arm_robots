# dual_arm_robots
# 双机械臂在ROS的搭建
## 1.控制两个真实机械臂（通过命名空间方式，两个机械臂完全独立）
+ roslaunch probot_bringup two_real_g603.launch

## 2.控制两个机械臂在同一个gazebo仿真中
+ roslaunch test1_moveit_config demo_gazebo.launch
