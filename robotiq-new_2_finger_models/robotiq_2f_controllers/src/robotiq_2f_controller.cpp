#include <pluginlib/class_list_macros.h>

#include <robotiq_2f_controllers/robotiq_2f_action_controller.h>

namespace position_controllers
{
  typedef gripper_action_controller::Robotiq2FActionController<hardware_interface::PositionJointInterface>
          Robotiq2FActionController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::Robotiq2FActionController, controller_interface::ControllerBase)