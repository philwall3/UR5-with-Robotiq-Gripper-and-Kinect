#ifndef ROBOTIQ_2F_HARDWARE____ROBOTIQ_2F_HW_H
#define ROBOTIQ_2F_HARDWARE____ROBOTIQ_2F_HW_H

#include <boost/scoped_ptr.hpp>

#include <std_msgs/Duration.h>

#include <urdf/model.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// KDL to solve the kinematics in transmission
/*#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>*/

namespace robotiq_2f_hardware
{

class ROBOTIQ2FHW : public hardware_interface::RobotHW
{
public:

	ROBOTIQ2FHW() {}
	virtual ~ROBOTIQ2FHW() {}

	/**
	 * @brief create
	 * @param name
	 * @param urdf_string
	 */
	void create(std::string name, std::string urdf_string);

	/**
	* @brief init Pure virtual function that needs to be implemented in the chosen interface
	 * @return
	 */
	virtual bool init() = 0;

	/**
	* @brief read Pure virtual function that needs to be implemented in the chosen interface
	 */
	virtual void read() = 0;

	/**
	* @brief write Pure virtual function that needs to be implemented in the chosen interface
	 */
	virtual void write() = 0;

	/**
	* @brief enforceLimits
	* @param period
	*/
	void enforceLimits(ros::Duration period);

	/**
	* @brief reset
	*/
	void reset();

	/**
	* @brief solveConstrainedConfiguration
	*/
	void solveConstrainedConfiguration();

	hardware_interface::JointStateInterface state_interface_;
	hardware_interface::EffortJointInterface effort_interface_;
	hardware_interface::PositionJointInterface position_interface_;

	int n_joints_;

	std::vector<std::string> joint_names_;

	std::vector<double>
	joint_position_,
	joint_position_prev_,
	joint_velocity_,
	joint_effort_,
	joint_position_command_,
	joint_effort_command_;

	std::vector<double>
	joint_lower_limits_,
	joint_upper_limits_,
	joint_effort_limits_;
};

} // robotiq_2f_hardware


#endif // ROBOTIQ_2F_HARDWARE____ROBOTIQ_2F_HW_H
