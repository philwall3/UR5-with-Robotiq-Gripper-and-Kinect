#ifndef ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_HW_USB_H
#define ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_HW_USB_H

#include <modbus.h>

#include <chrono>
#include <thread>

#include <control_toolbox/filters.h>

#include "robotiq_2f_hardware/robotiq_2f_hw.h"
#include "robotiq_2f_hw_usb/robotiq_2f_usb_comm.h"

#ifndef DEBUG
#define DEBUG 0
#endif

namespace robotiq_2f_hardware
{

class ROBOTIQ2FUSB : public ROBOTIQ2FHW
{
public:

	/**
	 * @brief init
	 * @return
	 */
	bool init()
	{
		// configure and connect to device
		ctx_ptr_ = NULL;
		ctx_ptr_ = modbus_new_rtu(port_.c_str(), 115200, 'N', 8, 1);
#if DEBUG
		modbus_set_debug(ctx_ptr_, TRUE);
#endif
		modbus_rtu_set_serial_mode(ctx_ptr_, MODBUS_RTU_RS485);
		modbus_set_slave(ctx_ptr_, server_id_);
		modbus_connect(ctx_ptr_);

		// init command structure with default values
		rq_cmd_.buffer[0] = ACTIVATION_CMD[0];
		rq_cmd_.buffer[1] = ACTIVATION_CMD[1];
		rq_cmd_.buffer[2] = ACTIVATION_CMD[2];

		// finally activate the device (handshake) and check
		int ra = modbus_write_registers(ctx_ptr_, CMD_ADDR, 9, ACTIVATION_CMD);
		if(ra < 0)
		{
			std::cout << "Couldn't perform an activation on the USB device" << std::endl;
			return false;
		}

		// the sleep is needed because the ready flag is turned true before
		// completing the initial movement after activation
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		bool ready = false;
		int timeout = 0;
		while(!ready)
		{
			modbus_read_registers(ctx_ptr_, MSR_ADDR, 9, rq_msr_.buffer);
			getStatus(rq_msr_, ready);

			timeout++;
			if (timeout > 50)
			{
				std::cout << "After 5seg, the USB device dind't get activated, quit." << std::endl;
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		return true;
	}

	/**
	 * @brief read
	 */
	void read()
	{
		int rc = modbus_read_registers(ctx_ptr_, MSR_ADDR, 9, rq_msr_.buffer);
		if(rc < 0)
		{
			ROS_FATAL("Couldn't read the last state on the USB device");
			exit(-1);
			return;
		}

		joint_position_prev_.at(0) = joint_position_.at(0);
		getPositionCurrent(rq_msr_, joint_position_.at(0), joint_effort_.at(0));
		joint_velocity_.at(0) = filters::exponentialSmoothing((joint_position_.at(0)-joint_position_prev_.at(0))/0.001,
															  joint_velocity_.at(0),
															  0.2);
	}

	/**
	 * @brief write
	 */
	void write()
	{
		setPositionEffort(joint_position_command_.at(0), 9.0, rq_cmd_);

		int rc = modbus_write_registers(ctx_ptr_, CMD_ADDR, 9, rq_cmd_.buffer);
		if(rc < 0)
		{
			ROS_FATAL("Couldn't write the last command on the USB device");
			exit(-1);
		}
	}

	/**
	 * @brief close This function resets the gripper, and then release the USB port.
	 */
	void close()
	{
		modbus_write_registers(ctx_ptr_, CMD_ADDR, 9, RESET_CMD);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		modbus_close(ctx_ptr_);
	}

	/**
	 * @brief reactivate This function resets to clear errors, and reactivate the gripper
	 */
	void reactivate()
	{
		int rs = modbus_write_registers(ctx_ptr_, CMD_ADDR, 9, RESET_CMD);
		if(rs < 0)
		{
			std::cout << "Couldn't perform a reset on the USB device" << std::endl;
			exit(-2);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		int ra = modbus_write_registers(ctx_ptr_, CMD_ADDR, 9, ACTIVATION_CMD);
		if(ra < 0)
		{
			std::cout << "Couldn't perform an activation on the USB device" << std::endl;
			exit(-2);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}

	/**
	 * @brief estop This function resets the gripper to clear all commands and errors
	 */
	void estop()
	{
		modbus_write_registers(ctx_ptr_, CMD_ADDR, 9, ESTOP_CMD);
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	}

	void setPort(std::string port){port_ = port;};

	void setServerID(int server_id){server_id_ = server_id;};

private:
	modbus_t *ctx_ptr_;

	std::string port_;
	int server_id_;

	robotiq_2f_hardware::rq_comm rq_cmd_;
	robotiq_2f_hardware::rq_comm rq_msr_;
};

} // robotiq_2f_hardware

#endif // ROBOTIQ_2F_HW_USB____ROBOTIQ_2F_HW_USB_H
