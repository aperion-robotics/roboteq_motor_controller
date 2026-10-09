#ifndef ROBOTEQ_HARDWARE_INTERFACE_HPP_
#define ROBOTEQ_HARDWARE_INTERFACE_HPP_

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "transmission_interface/transmission.hpp"

// ROS
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <serial/serial.h>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "roboteq_hardware_interface_parameters.hpp"

namespace roboteq_motor_controller_driver
{

// static const unsigned MAX_RECONNECT_ATTEMPTS = 10;

class RoboteqHardwareInterface : public hardware_interface::SystemInterface
{

public:
	RCLCPP_SHARED_PTR_DEFINITIONS(RoboteqHardwareInterface)
	virtual ~RoboteqHardwareInterface();

	hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
	hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
	hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
	// hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

	hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
	hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
	hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
  	hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;

private:
	// ROS params
	Params params_;

	// transmissions
	std::vector<std::shared_ptr<transmission_interface::Transmission>> position_transmissions_;
	std::vector<std::shared_ptr<transmission_interface::Transmission>> velocity_transmissions_;

	std::vector<double> position_commands_;
	std::vector<double> velocity_commands_;
	std::vector<double> joints_position_transmission_passthrough_;
	std::vector<double> actuators_position_transmission_passthrough_;
	std::vector<double> joints_velocity_transmission_passthrough_;
	std::vector<double> actuators_velocity_transmission_passthrough_;
	std::vector<double> joint_positions_;
	std::vector<double> joint_velocities_;

	// Roboteq query
	std::stringstream serial_query_;
	std::string position_query_;
	std::string velocity_query_;
	
	// Serial
	serial::Serial ser_;

	// resources switching aux vars
	std::vector<std::string> stop_modes_;
	std::vector<std::string> start_modes_;
	bool position_controller_running_;
	bool velocity_controller_running_;
};
} // namespace roboteq_motor_controller_driver

#endif // ROBOTEQ_HARDWARE_INTERFACE_HPP_