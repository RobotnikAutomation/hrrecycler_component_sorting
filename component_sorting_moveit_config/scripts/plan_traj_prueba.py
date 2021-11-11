#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import tf2_ros


moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('move_group_python_interface_tutorial')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
broadcaster = tf2_ros.StaticTransformBroadcaster()

pose_target = PoseStamped()
pose_target.header.frame_id = "robot_left_table_holder_approach_link"
#pose_target.pose.position.x = 0.20
pose_target.pose.position.z = 0.24
#pose_target.pose.orientation.x = 0
#pose_target.pose.orientation.y = 1
#pose_target.pose.orientation.z = 3.14
#pose_target.pose.orientation.w = 0
group.set_pose_target(pose_target)

print pose_target
group.go(wait=True)
rospy.sleep(5)


moveit_commander.roscpp_shutdown()
