#!/usr/bin/env python
# coding: utf8

import sys
import rospy
import rospkg, genpy
import yaml
import copy

import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
#from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
#from moveit_commander import roscpp_initialize, roscpp_shutdown
#from moveit_msgs.msg import RobotState, Grasp




def open_gripper():
	gripper_values = gripper.get_current_joint_values()
	#print "============ Joint values: ", gripper_values
	gripper_values[0] = 0
	gripper.set_joint_value_target(gripper_values)
	plan_open = gripper.plan()
	#rospy.sleep(3)
	gripper.execute(plan_open)
	gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	group.set_start_state_to_current_state() #WICHTIG
	rospy.loginfo("gripper opened")  

def close_gripper():	
	gripper_values = gripper.get_current_joint_values()
	#print "============ Joint values: ", gripper_values
	gripper_values[0] = 0.8040
	gripper.set_joint_value_target(gripper_values)
	plan_close = gripper.plan()
	#rospy.sleep(3)
	gripper.execute(plan_close)
	gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	group.set_start_state_to_current_state() #WICHTIG
	rospy.loginfo("gripper closed")

def almost_close_gripper():	
	gripper_values = gripper.get_current_joint_values()
	#print "============ Joint values: ", gripper_values
	gripper_values[0] = 0.6
	gripper.set_joint_value_target(gripper_values)
	plan_close = gripper.plan()
	#rospy.sleep(3)
	gripper.execute(plan_close)
	gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	group.set_start_state_to_current_state() #WICHTIG
	rospy.loginfo("gripper almost closed")   

def move_arm_one():
	pose_target = geometry_msgs.msg.Pose()	
	pose_target.position.x = -0.458138254168
	pose_target.position.y = -0.0137989226991
	pose_target.position.z = 0.333859798592
	pose_target.orientation.x = 0.0221718846011
	pose_target.orientation.y = -0.429881261108
	pose_target.orientation.z = 0.044288635466
	pose_target.orientation.w = 0.90152594286
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	#rospy.sleep(1)	
	group.execute(plan1)
	group.set_start_state_to_current_state() #WICHTIG
	#gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	rospy.loginfo("arm moved")  

def move_arm_two():
	pose_target = geometry_msgs.msg.Pose()	
	pose_target.position.x = -0.619355663979
	pose_target.position.y = -0.0299365439254 
	pose_target.position.z = 0.3156229882
	pose_target.orientation.x = 0.0208179904337
	pose_target.orientation.y = -0.394891524317
	pose_target.orientation.z = 0.0432710012193
	pose_target.orientation.w = 0.917472024506
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	#rospy.sleep(1)	
	group.execute(plan1)
	group.set_start_state_to_current_state() #WICHTIG
	#gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	rospy.loginfo("arm moved")  

def move_arm_three():
	pose_target = geometry_msgs.msg.Pose()	
	pose_target.position.x = -0.636781227553
	pose_target.position.y = -0.0316810780096
	pose_target.position.z = 0.332290095454
	pose_target.orientation.x = 0.0207586090949
	pose_target.orientation.y = -0.39487654276
	pose_target.orientation.z = 0.0433742089141
	pose_target.orientation.w = 0.917474944686
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	#rospy.sleep(1)	
	group.execute(plan1)
	group.set_start_state_to_current_state() #WICHTIG
	#gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	rospy.loginfo("arm moved")

def move_up():
	pose_target = geometry_msgs.msg.Pose()	
	pose_target.position.x = 0
	pose_target.position.y = 0.191449995733 
	pose_target.position.z = 1.00105899482
	pose_target.orientation.x = -0.707106769449
	pose_target.orientation.y = -0.00013118684414
	pose_target.orientation.z = 0
	pose_target.orientation.w = 0.707106777103
	group.set_pose_target(pose_target)
	plan1 = group.plan()
	#rospy.sleep(1)	
	group.execute(plan1)
	group.set_start_state_to_current_state() #WICHTIG
	#gripper.set_start_state_to_current_state() #evtl bei gripper nicht benötigt?
	rospy.loginfo("arm moved up")

def _adding_box():
	p = PoseStamped()
	p.header.frame_id = _robot.get_planning_frame()
	p.header.stamp = rospy.Time.now()

	p.pose.position.x = 0.0
	p.pose.position.y = 0.0
	p.pose.position.z = -0.01
	p.pose.orientation.w = 1.0

	_scene.add_box("floor", p, (7, 7, 0.02))
	return p.pose
'''
#Shift Pose funktioniert nicht besonders gut, da Pfad nie exakt gerade ist => Cartesian Path
def open_drawer():
	#Get the current pose of the end effector, add value to the corresponding axis (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target
	#def shift_pose_target(self, axis, value, end_effector_link = "") 
	group.shift_pose_target(0, 0.20, end_effector_link)
	group.go()
	#plan1.group.plan()

def close_drawer():
	#Get the current pose of the end effector, add value to the corresponding axis (0..5: X, Y, Z, R, P, Y) and set the new pose as the pose target
	#def shift_pose_target(self, axis, value, end_effector_link = "") 
	group.shift_pose_target(0, -0.20, end_effector_link)
	group.go()
	#plan1.group.plan()
'''
	
	
def open_drawer():
	waypoints = []
	waypoints.append(group.get_current_pose().pose)
	wpose = geometry_msgs.msg.Pose()
	wpose.orientation.w = waypoints[0].orientation.w
	wpose.orientation.x = waypoints[0].orientation.x
	wpose.orientation.y = waypoints[0].orientation.y
	wpose.orientation.z = waypoints[0].orientation.z
	wpose.position.x = waypoints[0].position.x + 0.2
	wpose.position.y = waypoints[0].position.y
	wpose.position.z = waypoints[0].position.z	

	waypoints.append(copy.deepcopy(wpose))
	(plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
	group.execute(plan3)


def close_drawer():
	waypoints = []
	waypoints.append(group.get_current_pose().pose)
	wpose = geometry_msgs.msg.Pose()
	wpose.orientation.w = waypoints[0].orientation.w
	wpose.orientation.x = waypoints[0].orientation.x
	wpose.orientation.y = waypoints[0].orientation.y
	wpose.orientation.z = waypoints[0].orientation.z
	wpose.position.x = waypoints[0].position.x - 0.2
	wpose.position.y = waypoints[0].position.y
	wpose.position.z = waypoints[0].position.z	

	waypoints.append(copy.deepcopy(wpose))
	(plan4, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
	group.execute(plan4)





if __name__ == "__main__":
	#roscpp_initialize(sys.argv)
	moveit_commander.roscpp_initialize(sys.argv)
	display_trajectory_publisher = rospy.Publisher(
		                      '/move_group/display_planned_path',
		                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)
	rospy.init_node('moveit_grasp_app', anonymous=True)
	rospy.loginfo("Starting grasp app")
	# add some code here

	_robot = moveit_commander.RobotCommander()
	_scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("manipulator")
	gripper = moveit_commander.MoveGroupCommander("gripper")
	
	rospy.sleep(1)

	group.set_max_acceleration_scaling_factor(0.2)
	group.set_max_velocity_scaling_factor(0.2)

	
	group.set_planer_id = "RRTkConfigDefault"
	group.set_planning_time(50)
	
        # Get the name of the end-effector link
	end_effector_link = group.get_end_effector_link()

	#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
	

	#move_group_python_interface()
	_adding_box()

	open_gripper()	
	move_arm_one()
	rospy.sleep(4)
	almost_close_gripper()
	rospy.sleep(2)
	move_arm_two()
	#rospy.sleep(2)
	#move_arm_three()
	#rospy.sleep(2)

	#close_gripper()
	#open_drawer()
	#rospy.sleep(6)
	#close_drawer()
	#rospy.sleep(4)
	#almost_close_gripper()

	#move_arm_two()
	#rospy.sleep(2)
	#move_arm_one()
	#rospy.sleep(2)
	#move_up()
	
	
	
	rospy.spin()
	moveit_commander.roscpp_shutdown() #vorher roscpp_shutdown()
	rospy.loginfo("Stopping grasp app")

