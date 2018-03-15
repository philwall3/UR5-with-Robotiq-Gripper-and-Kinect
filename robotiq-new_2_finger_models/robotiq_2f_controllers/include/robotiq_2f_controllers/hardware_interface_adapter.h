///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, SRI International
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of SRI International nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Sachin Chitta, Adolfo Rodriguez Tsouroukdissian

#ifndef GRIPPER_ACTION_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H
#define GRIPPER_ACTION_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

/**
 * \brief Helper class to simplify integrating the GripperActionController with different hardware interfaces.
 *
 * The GripperActionController outputs position while
 * it is supposed to work with either position or effort commands.
 *
 */
template <class HardwareInterface>
class HardwareInterfaceAdapter
{
public:
  bool init(hardware_interface::JointHandle& joint_handle, ros::NodeHandle& controller_nh)
  {
    return false;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  double updateCommand(const ros::Time&     time,
		       const ros::Duration& period,
		       double desired_position,
		       double desired_velocity,
		       double error_position,
		       double error_velocity,
		       double max_allowed_effort) { return 0.0;}

};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 */
template<>
class HardwareInterfaceAdapter<hardware_interface::PositionJointInterface>
{
public:
  HardwareInterfaceAdapter() : joint_handle_ptr_(0) {}

  bool init(hardware_interface::JointHandle& joint_handle, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handle_ptr_ = &joint_handle;

    return true;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  double updateCommand(const ros::Time&     /*time*/,
		       const ros::Duration& period,
		       double desired_position,
		       double desired_velocity,
		       double error_position,
		       double error_velocity,
		       double max_allowed_effort)
  {
    // Forward desired position to command
    (*joint_handle_ptr_).setCommand(desired_position);
    return max_allowed_effort;
  }

private:
  hardware_interface::JointHandle* joint_handle_ptr_;
};

#endif // header guard