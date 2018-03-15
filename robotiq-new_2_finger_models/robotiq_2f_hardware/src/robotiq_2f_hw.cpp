#include <robotiq_2f_hardware/robotiq_2f_hw.h>

namespace robotiq_2f_hardware
{

void ROBOTIQ2FHW::create(std::string name, std::string urdf_string)
{
	// set sizes
	n_joints_ = 1;
	joint_position_.resize(n_joints_);
	joint_position_prev_.resize(n_joints_);
	joint_effort_.resize(n_joints_);
	joint_velocity_.resize(n_joints_);

	joint_position_command_.resize(n_joints_);
	joint_effort_command_.resize(n_joints_);

	// set zeros
	joint_position_command_.at(0) = 0.0;
	joint_effort_command_.at(0) = 0.0;
	joint_position_.at(0) = 0.0;
	joint_position_prev_.at(0) = 0.0;
	joint_effort_.at(0) = 0.0;
	joint_velocity_.at(0) = 0.0;

	// set names
	//joint_names_.push_back( name + std::string("_right_driver_joint") ); //original	
	joint_names_.push_back( name + std::string("robotiq_85_left_knuckle_joint") ); //to work with model from robotiq_85_gripper from Stanley Innovation

	std::vector<transmission_interface::TransmissionInfo> all_transmissions;
	std::vector<transmission_interface::TransmissionInfo> transmissions;
	transmission_interface::TransmissionParser::parse(urdf_string, all_transmissions);

	// get this robot transmission's only
	for (int j = 0; j < n_joints_; ++j)
	{
		// std::cout << "Check joint " << joint_names_[j] << std::endl;
		std::vector<transmission_interface::TransmissionInfo>::iterator it = all_transmissions.begin();
		for(; it != all_transmissions.end(); ++it)
		{
			if (joint_names_[j].compare(it->joints_[0].name_) == 0)
			{
				transmissions.push_back( *it );
			}
	 	}
	}

	if( transmissions.empty() )
	{
		std::cout << "robotiq_2f_hardware: " << "There are no transmission in this robot, all are non-driven joints? " 
		<< std::endl;
		return;
	}

	// register interfaces
	for(int j=0; j < n_joints_; j++)
	{
		// check that this transmission has one joint
		if(transmissions[j].joints_.size() == 0)
		{
		std::cout << "robotiq_2f_hardware: " << "Transmission " << transmissions[j].name_
			<< " has no associated joints." << std::endl;
		continue;
		}
		else if(transmissions[j].joints_.size() > 1)
		{
		std::cout << "robotiq_2f_hardware: " << "Transmission " << transmissions[j].name_
			<< " has more than one joint, and they can't be controlled simultaneously"
			<< std::endl;
		continue;
		}

		std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

		if( joint_interfaces.empty() )
		{
		std::cout << "robotiq_2f_hardware: " << "Joint " << transmissions[j].joints_[0].name_ <<
			" of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
			"You need to, otherwise the joint can't be controlled." << std::endl;
		continue;
		}

		const std::string& hardware_interface = joint_interfaces.front();

		// create joint state interface for all joints
		state_interface_.registerHandle(hardware_interface::JointStateHandle(
										joint_names_[j], 
										&joint_position_[j], 
										&joint_velocity_[j], 
										&joint_effort_[j]));

		// Only Position interface is available for the robotiq 2f grippers

		hardware_interface::JointHandle joint_handle_position;
		joint_handle_position = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]), &joint_position_command_[j]);
		position_interface_.registerHandle(joint_handle_position);

		/*registerJointLimits(joint_names_[j],
						  joint_handle_effort, 
						  joint_handle_position,
						  joint_handle_velocity,
						  joint_handle_stiffness,
						  urdf_model, 
						  &joint_lower_limits_[j], &joint_upper_limits_[j],
						  &joint_lower_limits_stiffness_[j],
						  &joint_upper_limits_stiffness_[j],
						  &joint_effort_limits_[j]);*/

		
	}

	registerInterface(&state_interface_);
	registerInterface(&effort_interface_);
	registerInterface(&position_interface_);

	return;
}

}
