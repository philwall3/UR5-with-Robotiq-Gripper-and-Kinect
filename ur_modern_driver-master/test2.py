#!/usr/bin/env python

import sys
import rospy
import rospkg, genpy
import yaml
import copy

import moveit_commander
import geometry_msgs.msg
import geometry_msgs.msg
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Grasp
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String


def move_group_python_interface():
	print "============ Generating plan 1"
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1.0
	pose_target.position.x = 0.26345
	pose_target.position.y = -0.038236
	pose_target.position.z = 0.95528
	group.set_pose_target(pose_target)
	plan1 = group.plan()


	group.go(wait=True) #uncomment with real robot #moves robot arm
	#group.clear_pose_targets()

	## Then, we will get the current set of joint values for the group
	group_variable_values = group.get_current_joint_values()
	print "============ Joint values: ", group_variable_values



if __name__ == "__main__":
	#roscpp_initialize(sys.argv)
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_grasp_app', anonymous=True)
	rospy.loginfo("Starting grasp app")
	# add some code here

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("manipulator")
	
	#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
	rospy.sleep(1)

	move_group_python_interface()
	
	rospy.spin()
	roscpp_shutdown()
	rospy.loginfo("Stopping grasp app")
