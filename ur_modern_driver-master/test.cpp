#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(1);
	spinner.start();


	/* This sleep is ONLY to allow Rviz to come up */
	sleep(20.0);











}
