#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



def move_group_python_interface():
	## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
	print "============ Waiting for RVIZ..."
	rospy.sleep(10)
	print "============ Starting tutorial "

	## Getting Basic Information
	## ^^^^^^^^^^^^^^^^^^^^^^^^^
	##
	## We can get the name of the reference frame for this robot
	#print "============ Reference frame: %s" % group.get_planning_frame()

	## We can also print the name of the end-effector link for this group
	#print "============ Reference frame: %s" % group.get_end_effector_link()

	## We can get a list of all the groups in the robot
	print "============ Robot Groups:"
	#print robot.get_group_names()

	## Sometimes for debugging it is useful to print the entire state of the
	## robot.
	print "============ Printing robot state"
	#print robot.get_current_state()
	print "============"



if __name__=='__main__':
	try:
 
		rospy.on_shutdown(shutdown)
 
		#Initialisierung des MoveIt! Commanders
		moveit_commander.roscpp_initialize(sys.argv)
		#ruft die Funktion "move_group_python_interface" auf, die die Steine setzt
		rospy.init_node('move_group_python_interface', anonymous=True)
 		
		rospy.loginfo("blubb")

		## Instantiate a RobotCommander object.  This object is an interface to
		## the robot as a whole.
		robot = moveit_commander.RobotCommander()

		## Instantiate a PlanningSceneInterface object.  This object is an interface
		## to the world surrounding the robot.
		scene = moveit_commander.PlanningSceneInterface()
 
		#pillar = moveit_commander.MoveGroupCommander("pillar")
		#left_arm = moveit_commander.MoveGroupCommander("left_arm")
		#right_arm = moveit_commander.MoveGroupCommander("right_arm")
		#left_gripper = moveit_commander.MoveGroupCommander("left_gripper")
		#right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
		#Initialisierung Ende
		
		## Instantiate a MoveGroupCommander object.  This object is an interface
  		## to one group of joints.  In this case the group is the joints in the left
 		## arm.  This interface can be used to plan and execute motions on the left arm.
		group = moveit_commander.MoveGroupCommander("sdf")
 
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
		rospy.sleep(1)
 
		move_group_python_interface()
	except rospy.ROSInterruptException:
		pass
