#ifndef ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_USB_COMM_H
#define ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_USB_COMM_H

namespace robotiq_2f_hardware
{

// this is fixed for the usb mode
const int CMD_ADDR = 0x03E8;
const int MSR_ADDR = 0x07D0;

// the activation cmd also contains the default values for the GoTo command
// speed in device at 100%, the decimation is done by the joint_trajectory controller
// effort in device at 0
const uint16_t ACTIVATION_CMD[3] = {(1 + (1 << 3) + (0 << 4)) << 8, 0x0000, (0xFF << 8) + 0xFF};
// the reset command clears all values, clears all faults
// and stops any current motion
const uint16_t RESET_CMD[3] = {0x0, 0x0, 0x0};
// the emergency stop command makes all other commands to stop, and perform a slow
// openning of the gripper. A reset, and activation commands are required to re-enable
// the gripper
const uint16_t ESTOP_CMD[3] = {(1 + (0 << 3) + (1 << 4)) << 8, 0x0, 0x0};

// max joint value in rad, and effort in Nm, from urdf could be extracted too
const double POS_MAX = 50 * 3.141592 / 180;
const double VEL_MAX = 100 * 3.141592 / 180;
const double EFF_MAX = 10;

union rq_comm
{
	uint16_t buffer[3];
	uint8_t values[6];
};

void setPosition(const double &pos, rq_comm &cmd)
{
	uint8_t u_pos = std::lround( (255*pos/POS_MAX));
	cmd.buffer[1] = (0x0 << 8) + u_pos;
};


void setPositionEffort(const double &pos, const double &eff, rq_comm &cmd)
{
	uint8_t u_pos = std::lround( (255*pos/POS_MAX));
	uint8_t u_eff = std::lround( (255*eff/EFF_MAX));
	cmd.buffer[1] = (0x0 << 8) + u_pos;
	cmd.buffer[2] = (0xFF << 8) + u_eff; // 0xFFFF;
};

void setPositionVelocityEffort(const double &pos, const double &vel, const double &eff, rq_comm &cmd)
{
	uint8_t u_pos = std::lround( (255*pos/POS_MAX));
	uint8_t u_vel = std::lround( (255*vel/VEL_MAX));
	uint8_t u_eff = std::lround( (255*eff/EFF_MAX));
	cmd.buffer[1] = (0x0 << 8) + u_pos;
	cmd.buffer[2] = (u_vel << 8) + u_eff;
};

void getStatus(const rq_comm &msr, bool &status)
{
	uint8_t u_act = (msr.buffer[0] & 0xFF00) >> 8;
	status = (u_act >> 0) & 0x01;
}

void getPositionCurrent(const rq_comm &msr, double &pos, double &cur)
{
	uint8_t u_pos = (msr.buffer[2] & 0xFF00) >> 8;
	uint8_t u_cur = (msr.buffer[2] & 0x00FF);
	pos = (double)(u_pos/255.0f)*POS_MAX;
	cur = (double)(u_cur/255.0f)*EFF_MAX;
}

} // robotiq_2f_hardware

#endif // ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_USB_COMM_H
