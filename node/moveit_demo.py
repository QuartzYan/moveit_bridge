# -*- coding: utf-8 -*-
#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

#初始化moveit_commander和rospy
print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

#实例化RobotCommander对象，这个接口是机器人总入口
robot = moveit_commander.RobotCommander()

#实例化PlanningSceneInterface对象，这个接口围绕机器人的世界
scene = moveit_commander.PlanningSceneInterface()

#实例化MoveGroupCommander对象
group = moveit_commander.MoveGroupCommander("ur5_arm")

#创建DisplayTrajectory发布器，可以得到轨迹在Rviz总实现可视化。
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

#等待Rviz初始化
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "

#打印参考系的名称
print "============ Reference frame: %s" % group.get_planning_frame()
#打印这个组的末端执行器的连接名称
print "============ Reference frame: %s" % group.get_end_effector_link()
#获得机器人的所有组
print "============ Robot Groups:"
print robot.get_group_names()
#打印机器人的状态
print "============ Printing robot state"
print robot.get_current_state()
print "============"

#规划姿态目标
print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = 0
pose_target.position.z = 1.5
group.set_pose_target(pose_target)

#调用规划器计算规划并在Rviz里显示
plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)

#移动到姿态目标
group.go(wait=True)

