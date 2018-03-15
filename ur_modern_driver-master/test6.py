#!/usr/bin/env python

import sys
import rospy
import rospkg, genpy
import yaml
import copy

import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
#from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
#from moveit_commander import roscpp_initialize, roscpp_shutdown
#from moveit_msgs.msg import RobotState, Grasp
#from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String


def move_group_python_interface():
	group.set_start_state_to_current_state() #WICHTIG	
	display_trajectory_publisher = rospy.Publisher(
		                      '/move_group/display_planned_path',
		                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)

	## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
	print "============ Waiting for RVIZ..."
	rospy.sleep(5)
	print "============ Starting tutorial "



	print "============ Generating plan 1"
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1.0
	pose_target.position.x = 0.26345
	pose_target.position.y = -0.038236
	pose_target.position.z = 0.95528
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	rospy.sleep(10)	
	#group.clear_pose_targets()

	#group.go(wait=True) #uncomment with real robot #moves robot arm
	group.execute(plan1)
	rospy.sleep(5)
	group.set_start_state_to_current_state() #WICHTIG


	## Then, we will get the current set of joint values for the group
	group_variable_values = group.get_current_joint_values()
	print "============ Joint values: ", group_variable_values


	#---------------------------------------------------------------------------------------
	##ZWEITE BEWEGUNG!!
	#group_variable_values[0] = 0
	#group_variable_values[1] = -1.570796314870016
	#group_variable_values[2] = 0
	#group_variable_values[3] = -1.5707600752459925
	#group_variable_values[4] = 0
	#group.set_joint_value_target(group_variable_values)
	
	#plan2 = group.plan()
	#rospy.sleep(5)
	#group.execute(plan2)	
	#rospy.sleep(5)
	#group.set_start_state_to_current_state() #WICHTIG


	#------------------------------------------------------------------------------------------
	##DRITTE BEWEGUNG
	start_pose = group.get_current_pose().pose
	waypoints = []

	# start with the current pose
	waypoints.append(group.get_current_pose().pose)
	print "============ current Waypoints: ", waypoints

	# first orient gripper and move forward (+x)
	#wpose = geometry_msgs.msg.Pose()
	#wpose.orientation.w = 0.7
	#wpose.position.x = waypoints[0].position.x -0.1 #+ 0.012
	#wpose.position.y = waypoints[0].position.y -0.1 #-0.05
	#wpose.position.z = waypoints[0].position.z #-0.12
	#waypoints.append(copy.deepcopy(wpose))
	#print "============ new Waypoints: ", waypoints

	wpose = geometry_msgs.msg.Pose()
	#wpose = copy.deepcopy(start_pose)
	wpose.orientation.x = waypoints[0].orientation.x
	wpose.orientation.y = waypoints[0].orientation.y
	wpose.orientation.z = waypoints[0].orientation.z
	wpose.orientation.w = 1
	wpose.position.x = waypoints[0].position.x -0.26 #+ 0.012
	wpose.position.y = waypoints[0].position.y -1.1 #-0.05
	wpose.position.z = waypoints[0].position.z -0.04 #-0.12
	waypoints.append(copy.deepcopy(wpose))
	print "============ new Waypoints: ", waypoints

	# second move down
	#wpose.position.z += 0.10
	#wpose.position.x = waypoints[0].position.x + 1.8
	#wpose.position.y = waypoints[0].position.y -1.5
	#waypoints.append(copy.deepcopy(wpose))

	fraction = 0.0
	maxtries = 100
	attempts = 0

	group.set_start_state_to_current_state() #WICHTIG

	while fraction < 1.0 and attempts < maxtries:
		(plan3, fraction) = group.compute_cartesian_path(
		                     waypoints,   # waypoints to follow
		                     0.1,        # eef_step
		                     0.0,         # jump_threshold
			             True) 	  # avoid_collisions
		attempts += 1
		plan3= group.retime_trajectory(robot.get_current_state(), plan3, 0.2)
		if attempts % 10 == 0:
		     rospy.loginfo("Still trying after " + str(attempts) + " attempts..")

	if fraction == 1.0:
		rospy.loginfo("Path computed successfully. Moving the arm.")

		group.execute(plan3)

		rospy.loginfo("Path execution complete.")
	else:
		rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")


	#print "============ new PLAN: ", plan3
	#rospy.sleep(3)
	
	#print "============ new PLAN2: ", plan3
	#rospy.sleep(3)	
	#group.execute(plan3)




if __name__ == "__main__":
	#roscpp_initialize(sys.argv)
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_grasp_app', anonymous=True)
	rospy.loginfo("Starting grasp app")
	# add some code here

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("manipulator")
	
	rospy.sleep(1)

	group.set_max_acceleration_scaling_factor(0.2)
	group.set_max_velocity_scaling_factor(0.2)

	
	group.set_planer_id = "RRTkConfigDefault"
	
	#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
	

	move_group_python_interface()
	
	rospy.spin()
	moveit_commander.roscpp_shutdown() #vorher roscpp_shutdown()
	rospy.loginfo("Stopping grasp app")
