#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations


from std_msgs.msg import String
from moveit_bridge.msg import pose



class MoveitBridge:
  def __init__(self, group_name):
    moveit_commander.roscpp_initialize(sys.argv)
    #实例化RobotCommander对象，这个接口是机器人总入口
    self._robot = moveit_commander.RobotCommander()
    #实例化PlanningSceneInterface对象，这个接口围绕机器人的世界
    self._scene = moveit_commander.PlanningSceneInterface()
    #实例化MoveGroupCommander对象
    self._group = moveit_commander.MoveGroupCommander(group_name)
    #创建DisplayTrajectory发布器，可以得到轨迹在Rviz总实现可视化。
    self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

    self._sub = rospy.Subscriber("moveit_bridge_pose", pose, self.callBack, queue_size=1)

    #打印参考系的名称
    print "============ Reference frame: %s" % self._group.get_planning_frame()
    #打印这个组的末端执行器的连接名称
    print "============ Reference frame: %s" % self._group.get_end_effector_link()
    #获得机器人的所有组
    #print "============ Robot Groups:"
    #print self._robot.get_group_names()
    #打印机器人的状态
    #print "============ Printing robot state"
    #print self._robot.get_current_state()
    #print "============"


  def callBack(self, msg):
    pose_target = geometry_msgs.msg.Pose()
    x,y,z,w = tf.transformations.quaternion_from_euler(msg.roll, msg.pitch, msg.yaw)
    pose_target.orientation.x = x
    pose_target.orientation.y = y
    pose_target.orientation.z = z
    pose_target.orientation.w = w
    pose_target.position.x = msg.x
    pose_target.position.y = msg.y
    pose_target.position.z = msg.z
    self._group.set_pose_target(pose_target)

    #调用规划器计算规划并在Rviz里显示
    plan = self._group.plan()

    print "============ Waiting while RVIZ displays plan..."
    rospy.sleep(5)
    if(msg.realGo):
      #移动到姿态目标
      self._group.set_max_velocity_scaling_factor(0.1)#关节的最大速度,不是末端执行器点的速度
      self._group.set_max_acceleration_scaling_factor(0.4)
      self._group.go(wait=True)
         

def main():
    rospy.init_node("moveit_bridge")
    group_name = rospy.get_param("group_name", default="ur5_arm")

    mb = MoveitBridge(group_name)
    
    rospy.spin()


if __name__ == "__main__":
    main()