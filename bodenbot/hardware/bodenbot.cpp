// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bodenbot/bodenbot.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}

////////////////////////////////////////////////

namespace bodenbot {

class ServoInterface {
public:
	ServoInterface(const char* deviceFile) {
		fd_ = open(filename, O_WRONLY | O_NOCTTY | O_SYNC);
		if (fd_ < 0)
		{
			throw std::runtime_error("Invalid port!");
		}
		struct termios tty;
		if (tcgetattr(fd_, &tty) != 0) 
		{
			throw std::runtime_error("Failed to make termios");
		}
		cfsetispeed(&tty, speed);
		cfsetospeed(&tty, speed);
	
		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
		tty.c_iflag &= ~IGNBRK; // disable break processing
		tty.c_lflag = 0; // no signaling chars, no echo, no
						 // canonical processing
		tty.c_oflag = 0; // no remapping, no delays
		tty.c_cc[VMIN] = 0; // read doesn't block
		tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
	
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	
		tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
								 // enable reading
		tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
		tty.c_cflag &= ~CSTOPB;
		tty.c_cflag &= ~CRTSCTS;
	
		if (tcsetattr(fd_, TCSANOW, &tty) != 0) 
		{
			throw std::runtime_error("tcsetattr failed!");
		}
	}
	~ServoInterface() {
		close(fd_);
	}
	void write_angle(int data)
	{
		std::string s = std::to_string(data);
		write(fd_, s.c_str(), s.length());
	}
	int read_angle()
	{
	    const size_t size = sizeof(int);
		char buffer[size];
		read(fd, buffer, size);
		int data = *(int*)buffer;
		std::cout << data << '\n';
		return data;
	}
private:
	int fd_;
};

hardware_interface::CallbackReturn SerialController::on_init(const hardware_interface::HardwareInfo &info) {
	// If system interface setup does not work return error
	if (hardware_interface::SystemInterface::on_init(info) != 
		hardware_interface::CallbackReturn::SUCCESS) {

    	return hardware_interface::CallbackReturn::ERROR;
	}

	// configuration parameters for hardware
	#define DEBUG_PARAM(message, name) { \
  		RCLCPP_INFO( /*Logs at info level*/  \
			rclcpp::get_logger("SerialController"), \
			message, \
        	info_.hardware_parameters[name].c_str() \
		); \
	}

	// Set up Debugs
  	DEBUG_PARAM("debug: %s", "debug");
  	DEBUG_PARAM("mock hardware: %s", "mock_hardware");
  	DEBUG_PARAM("i2c_file: %s", "i2c_file");
  	DEBUG_PARAM("i2c_address: %s", "i2c_address");
  	// parameter for velocity_max_accel

	#undef DEBUG_PARAM

	// Reads data from bodenbot.ros2_control.xacro
  	debug_ = info_.hardware_parameters["debug"] == "True";
  	mock_hardware_ = info_.hardware_parameters["mock_hardware"] == "True";
  	i2c_file_ = info_.hardware_parameters["i2c_file"];
  	i2c_address_ = std::atoi(info_.hardware_parameters["i2c_address"].c_str());

	// Set sizes of vectors
	std::size_t jointSize = info_.joints.size();
	double NaN = std::numeric_limits<double>::quiet_NaN();

  	hw_velocities_.resize(jointSize, NaN);
  	hw_commands_.resize(jointSize, NaN);
  	hw_positions_.resize(jointSize, NaN);

  	for (hardware_interface::ComponentInfo &joint : info_.joints) {
    	// we only have velocity command and feedback
		// Joint Command Interfaces MUST only have one element.
    	if (joint.command_interfaces.size() != 1) {
      		RCLCPP_FATAL(
				rclcpp::get_logger("SerialController"),
        		"Joint '%s' has %zu command interfaces found. 1 expected.",
          		joint.name.c_str(), joint.command_interfaces.size()
			);
      		return hardware_interface::CallbackReturn::ERROR;
    	}

		// Command interface MUST be velocity
		if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      		RCLCPP_FATAL(
        		rclcpp::get_logger("SerialController"),
        		"Joint '%s' have '%s' as first state interface. '%s' expected.",
        		joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        		hardware_interface::HW_IF_VELOCITY
			);
      		return hardware_interface::CallbackReturn::ERROR;
    	}
		// Joint State Interfaces MUST be of size 2
    	if (joint.state_interfaces.size() != 2) {
      		RCLCPP_FATAL(
				rclcpp::get_logger("SerialController"),
                "Joint '%s' has %zu state interface. 2 expected.",
                joint.name.c_str(), joint.state_interfaces.size()
			);
      		return hardware_interface::CallbackReturn::ERROR;
    	}
		// First State Interface MUST be velocity
    	if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      		RCLCPP_FATAL(
          		rclcpp::get_logger("SerialController"),
          		"Joint '%s' have '%s' as first state interface. '%s' expected.",
          		joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          		hardware_interface::HW_IF_VELOCITY
			);
      		return hardware_interface::CallbackReturn::ERROR;
    	}
		// Second State Interface MUST be position
    	if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION) {
      		RCLCPP_FATAL(
          		rclcpp::get_logger("SerialController"),
          		"Joint '%s' have '%s' as first state interface. '%s' expected.",
          		joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          		hardware_interface::HW_IF_POSITION
			);
      		return hardware_interface::CallbackReturn::ERROR;
    	}

		// Log joint as valid
    	RCLCPP_INFO (
			rclcpp::get_logger("SerialController"),
            "Found joint '%s' with id '%s' and reversed '%s'",
            joint.name.c_str(), joint.parameters["id"].c_str(),
            joint.parameters["reversed"].c_str()
		);

		// Add motor id
    	motor_id_.emplace_back(std::atoi(joint.parameters["id"].c_str()));
		// Add if joint is reversed
    	reversed_.emplace_back(joint.parameters["reversed"] == "true");
  	}
  	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SerialController::export_state_interfaces() {
	std::vector<hardware_interface::StateInterface> state_interfaces;

	// Foreach joint created two state interfaces, one velocity and one position
	for (auto i = 0u; i < info_.joints.size(); i++) {
    	state_interfaces.emplace_back(
			hardware_interface::StateInterface(
        		info_.joints[i].name, 
				hardware_interface::HW_IF_VELOCITY,
        		&hw_velocities_[i]
			)
		);
    	state_interfaces.emplace_back(
			hardware_interface::StateInterface(
        		info_.joints[i].name, 
				hardware_interface::HW_IF_POSITION,
        		&hw_positions_[i]
			)
		);
	}

	// Log export
  	RCLCPP_INFO(
		rclcpp::get_logger("SerialController"),
        "Exported %zu state interfaces", 
		state_interfaces.size()
	);

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SerialController::export_command_interfaces() {
	std::vector<hardware_interface::CommandInterface> command_interfaces;

	// Foreach joint create velocity command interface
	for (auto i = 0u; i < info_.joints.size(); i++) {
    	command_interfaces.emplace_back(
			hardware_interface::CommandInterface(
        		info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        		&hw_commands_[i]
			)
		);
  	}

	// Log serial controller export
  	RCLCPP_INFO(
		rclcpp::get_logger("SerialController"),
        "Exported %zu command interfaces", 
		command_interfaces.size()
	);

  	return command_interfaces;
}

hardware_interface::CallbackReturn SerialController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
	// Log that its activating
	RCLCPP_INFO(
		rclcpp::get_logger("SerialController"),
        "Activating ...please wait..."
	);

	// Set all velocities to zero on activation
	for (auto i = 0u; i < hw_velocities_.size(); i++) {
    	hw_velocities_[i] = 0;
  	}

	// IF using real hardware use SerialInterface
  	if (!mock_hardware_) {
    	serial_interface_ = new SerialInterface(i2c_file_, i2c_address_);
  	} 
	else {
    	serial_interface_ = NULL;
		// Log that this is NOT using real hardware
    	RCLCPP_INFO(
			rclcpp::get_logger("SerialController"),
            "Not writing to device"
		);
  	}

	// Foreach joint set velocities, commands, and positions to zero.
  	for (auto i = 0u; i < info_.joints.size(); i++) {
    	hw_commands_[i] = 0;
    	hw_velocities_[i] = 0;
    	hw_positions_[i] = 0;
  	}

	// Log Success
	RCLCPP_INFO(
		rclcpp::get_logger("SerialController"),
        "Successfully activated!"
	);

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  	// BEGIN: This part here is for exemplary purposes - Please do not copy to
  	// your production code
	// Log deactivation
  	RCLCPP_INFO(
		rclcpp::get_logger("SerialController"),
    	"Deactivating ...please wait..."
	);

	// If using hardware
  	if (serial_interface_) {
    	serial_interface_->close_connection();
    	delete serial_interface_;
    	serial_interface_ = NULL;
  	}

	// Log deactivation
  	RCLCPP_INFO(
		rclcpp::get_logger("SerialController"),
        "Successfully deactivated!"
	);

  	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SerialController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
	// update state interface for steer joint
  	for (auto i = 0u; i < info_.joints.size(); i++) {
		// If using hardware:
    	if (serial_interface_) {
      		int id = motor_id_[i];
      		try {
        		double velocity = serial_interface_->read_vel(id);
        		if (reversed_[i]) {
          			velocity = -velocity;
        		}
        		hw_velocities_[i] = velocity;
      		} 
			catch (std::exception const &e) {
				// If cannot read velocity log error
        		RCLCPP_ERROR(rclcpp::get_logger("SerialController"), e.what());
      		}
    	}

		// Update wheel positions
    	hw_positions_[i] += hw_velocities_[i] * period.seconds();
    	hw_positions_[i] = std::fmod(hw_positions_[i], 2 * M_PI);

    	if (debug_) {
      		RCLCPP_INFO(
				rclcpp::get_logger("SerialController"),
                "Read [%ld] (%f) (%f)", motor_id_[i], hw_velocities_[i],
                hw_positions_[i]
			);
    	}
  	}

  	return hardware_interface::return_type::OK;
}

hardware_interface::return_type bodenbot::SerialController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
	// Foreach joint
	for (auto i = 0u; i < info_.joints.size(); i++) {
    	int id = motor_id_[i];
    	double velocity = hw_commands_[i];
    	if (reversed_[i]) {
      		velocity = -velocity;
    	}

    	if (debug_) {
      		RCLCPP_INFO(
				rclcpp::get_logger("SerialController"), 
				"Write [%d] (%f)", 
				id,
                velocity
			);
    	}
		// Do not continue if no hardware
    	if (!serial_interface_) {
      		hw_velocities_[i] = velocity;
      		continue;
    	} 
		else { // Sets velocity to value in hardware commands
      		try {
        		serial_interface_->write_vel(id, velocity);
      		} 
			catch (std::exception const &e) {
        		RCLCPP_ERROR(rclcpp::get_logger("SerialController"), e.what());
      		}
    	}
  	}

  	return hardware_interface::return_type::OK;
}

} // namespace bodenbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	bodenbot::SerialController,
    hardware_interface::SystemInterface
)
