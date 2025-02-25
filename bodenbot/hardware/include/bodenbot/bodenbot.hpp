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

#ifndef BODENBOT_SYSTEM_HPP_
#define BODENBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "bodenbot/visibility_control.h"

namespace bodenbot {

/**
 * Class implemented in bodenbot.cpp that will handle interaction with hardware.
 */
class SerialInterface;

/**
 * Loads data from file to generate rover. Can then control and read speed of wheels.
 * IS the rover (sorta).
 */
class SerialController : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SerialController)

    /**
     * Loads hardware config from bodenbot.ros2_control.xacro
     * Initializes logging system
     * 
     * \param[in] info Data used to initialize SystemInterface with (The parent class)
     * 
     * \return Whether or not initialization was successful
     */
    BODENBOT_PUBLIC hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    /**
     * \return List of state interfaces
     */
    BODENBOT_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * \return List of command interfaces
     */
    BODENBOT_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
     * Activates Rover, initializes data
     * Sets up SerialInterface to interact with hardware if built for the purpose.
     * 
     * \param[in] previous_state Unused parameter
     * 
     * \return Whether or not activation was successful
     */
    BODENBOT_PUBLIC hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * Deactivates rover, removes SerialInterface if initialized
     * 
     * \param[in] previous_state Unused parameter
     * 
     * \return Whether or not deactivation was successful
     */
    BODENBOT_PUBLIC hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * Reads the positions of the motors and stores them in hw_positions_
     * 
     * \param[in] time Unused parameter
     * \param[in] period The measured time taken by the last control loop iteration
     */
    BODENBOT_PUBLIC hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * Sets motor speed
     * 
     * \param[in] time Unused param
     * \param[in] period Unused param
     */
    BODENBOT_PUBLIC hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Whether or not to perform special debug logging
    bool debug_;
    // Whether or not this is running on real hardware
    bool mock_hardware_;
    // Device driver
    std::string i2c_file_;
    // Address to write/read to device at
    int i2c_address_;

    // Where hardware interactions takeplace
    SerialInterface* serial_interface_;

    // Hardware/Motor info
    std::vector<double> hw_commands_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_positions_;
    std::vector<size_t> motor_id_;
    std::vector<bool> reversed_;
};

}  // namespace smokey

#endif  // BODENBOT__DIFFBOT_SYSTEM_HPP_
