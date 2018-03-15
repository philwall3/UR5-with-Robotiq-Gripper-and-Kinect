# UR5-with-Robotiq-Gripper-and-Kinect
This package enables an UR5 arm with a Robotiq 85 Gripper to be used with ros_control and MoveIt!. A Microsoft Kinect is used to create an Octomap.

It contains code from those repositories:

https://github.com/JenniferBuehler/common-sensors

https://github.com/beta-robots/robotiq

https://github.com/waypointrobotics/robotiq_85_gripper/

https://github.com/ThomasTimm/ur_modern_driver/

https://github.com/ros-industrial/universal_robot

# __Simulation__

To start a gazebo simulation ``myrobot2_gazebo_simulation.launch`` in ``myrobot2_usage`` has to be launched with ``roslaunch myrobot2_usage myrobot2_gazebo_simulation.launch``.

The position of the Kinect can be changed in ``common_sensors/urdf/sensors/kinect_properties.urdf.xacro``.

# __real robot__

In Rviz only the "Plan and Execute" button can be used to control the gripper. "Plan" and "Execute" do not work.


__without ODROID__

First connect the gripper to the computer. Then the gripper has to be activated. Do a ``roslaunch myrobot2_usage activate_gripper.launch`` and first enter "r" for reset and then "a" for activate. Then stop it. Now the gripper can be controlled.

Then the robot is brought up with ``roslaunch myrobot2_usage myrobot2_real_robot.launch``.

If Kinect is used, launch it with ``roslaunch myrobot2_usage kinect.launch``.


__with ODROID__

All needed packages have to be present on the ODROID.

Gripper (and Kinect) are connected to the ODROID. Then the gripper needs to be activated. Do a ``roslaunch myrobot2_usage activate_gripper_odroid.launch`` and first enter "r" for reset and then "a" for activate. Then stop it. Now the gripper can be controlled.

Then the robot is brought up with ``roslaunch myrobot2_usage myrobot2_real_robot_odroid.launch``.

If Kinect is used, launch it with ``roslaunch myrobot2_usage kinect_odroid.launch``.

Attention: To use roslaunch with remote machines (in this case an ODROID) the needed information in some files has to be correctly entered, otherwise it won't work. Those files are ``myrobot2_bringup_ros_control_odroid.launch``, ``activate_gripper_odroid.launch``, ``kinect_odroid.launch``. Also the bash files on the computer and ODROID have to be specified, like explained here:

https://web.archive.org/web/20160112224913/http://pharos.ece.utexas.edu:80/wiki/index.php/Running_ROSNodes_in_several_robots_simultaneously_using_ROSLaunch

https://answers.ros.org/question/201415/launch-file-to-run-multiple-nodes-in-two-machines-problem-with-environment-file/

http://wiki.ros.org/roslaunch/XML/machine
