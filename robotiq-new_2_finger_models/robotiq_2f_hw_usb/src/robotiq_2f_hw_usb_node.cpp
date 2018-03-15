#include <mutex>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "robotiq_2f_hw_usb/robotiq_2f_hw_usb.hpp"

bool isReactivateRequested = false;
bool isStopPressed = false;
bool wasStopHandled = true;
void eStopCB(const std_msgs::BoolConstPtr& e_stop_msg)
{
	isStopPressed = e_stop_msg->data;
}

// Get the URDF XML from the parameter server
// ToDO: Consider to include this function in an share_place
std::string getURDF(ros::NodeHandle &model_nh, std::string param_name)
{
	std::string urdf_string;
	std::string robot_description = "/robot_description";

	ros::Duration wait(1.0);

	// search and wait for robot_description on param server
	while (urdf_string.empty())
	{
		model_nh.getParam(param_name, urdf_string);
		ROS_INFO_NAMED("ROBOTIQ2FUSB", "ROBOTIQ2FUSB node is waiting for model"
			" URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

		if(!urdf_string.empty())
		{
			ROS_INFO("URDF found!!");
			break;
		}
		wait.sleep();
	}
	ROS_DEBUG_STREAM_NAMED("ROBOTIQ2FUSB", "Received URDF from param server, parsing...");

	return urdf_string;
}

bool reactivateCallback(std_srvs::TriggerRequest& __req, std_srvs::TriggerResponse& __res)
{
	isReactivateRequested = true;
	while(isReactivateRequested)
		ros::Duration(1.0).sleep();
	__res.success = true;
	__res.message = "The gripper has been reactivated";
	return true;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "robotiq_2f_hardware", ros::init_options::NoSigintHandler);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::NodeHandle rq2f_nh;

	// get params or give default values
	std::string port;
	int server_id;
	std::string name;
	rq2f_nh.param("port", port, std::string("/dev/ttyUSB0"));
	rq2f_nh.param("server_id", server_id, 9 );
	rq2f_nh.param("name", name, std::string("simple_gripper_"));

	// advertise the e-stop topic
	ros::Subscriber estop_sub = rq2f_nh.subscribe(rq2f_nh.resolveName("emergency_stop"), 1, eStopCB);

	// advertise the reactivate service
	ros::ServiceServer reactivate_srv = rq2f_nh.advertiseService("reactivate", reactivateCallback);

	// get the general robot description, the lwr class will take care of parsing what's useful to itself
	std::string urdf_string = getURDF(rq2f_nh, "/robot_description");

	// construct and start the Robotiq 2F gripper using the USB interface 
	robotiq_2f_hardware::ROBOTIQ2FUSB rq2f_hw;
	rq2f_hw.create(name, urdf_string);
	rq2f_hw.setPort(port);
	rq2f_hw.setServerID(server_id);

	if(!rq2f_hw.init())
	{
		ROS_FATAL_NAMED("robotiq_2f_hardware", "Could not initialize USB interface");
		return -1;
	}

	// timer variables
	struct timespec ts = {0, 0};
	ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
	ros::Duration period(1.0);

	//the controller manager
	controller_manager::ControllerManager manager(&rq2f_hw, rq2f_nh);

	// TODO rate so it does not runs crazy
	ros::Rate rate(10);

	while( ros::ok() )
	{
		if(!isReactivateRequested)
		{
			if (!clock_gettime(CLOCK_MONOTONIC, &ts))
			{
				now.sec = ts.tv_sec;
				now.nsec = ts.tv_nsec;
				period = now - last;
				last = now;
			}
			else
			{
				ROS_FATAL("Failed to poll realtime clock!");
				break;
			}

			rq2f_hw.read();

			bool resetControllers;
			if(!wasStopHandled && !resetControllers)
			{
				ROS_WARN("E-STOP HAS BEEN PRESSED: Controllers will be restarted, but the robot won't move until you release the E-Stop");
				ROS_WARN("You need to reactivate the gripper calling the service /reactivate, and then release the EStop");
				ROS_WARN("HOW TO RELEASE E-STOP: rostopic pub -r 10 /NAMESPACE/emergency_stop std_msgs/Bool 'data: false'");
				resetControllers = true;
				wasStopHandled = true;
				rq2f_hw.estop();
			}

			if( isStopPressed )
			{
				wasStopHandled = false;
			}
			else
			{
				resetControllers = false;
				wasStopHandled = true;
			}

			manager.update(ros::Time::now(), period, resetControllers);

			rq2f_hw.write();
			rate.sleep();
		}
		else
		{
			rq2f_hw.reactivate();
			isReactivateRequested = false;
		}
	}

	ROS_INFO("Stopping the Robotiq 2F USB device...");
	rq2f_hw.close();

	// give time to controllers to unload
	ros::Duration(1.0).sleep();

	ROS_INFO("Stopping spinner...");
	spinner.stop();

	ROS_INFO("Bye !!!");

	return 0;
}
