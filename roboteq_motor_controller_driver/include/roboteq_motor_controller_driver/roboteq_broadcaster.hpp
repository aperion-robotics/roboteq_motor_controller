#ifndef ROBOTEQ_BROADCASTER_HPP_
#define ROBOTEQ_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "roboteq_hardware_interface_parameters.hpp"
#include "roboteq_motor_controller_msgs/msg/query.hpp"

namespace roboteq_broadcaster
{

class RoboteqBroadcaster : public controller_interface::ControllerInterface
{

public:
	RCLCPP_SHARED_PTR_DEFINITIONS(RoboteqBroadcaster)

	controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

	controller_interface::CallbackReturn on_init() override;
	controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
	// ROS params
	std::shared_ptr<roboteq_motor_controller_driver::ParamListener> param_listener_;
	roboteq_motor_controller_driver::Params params_;

	// Publishers
  	std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<roboteq_motor_controller_msgs::msg::Query>>> realtime_roboteq_publishers_;
	std::vector<int> cumulative_state_interface_index;
};
} // namespace roboteq_broadcaster

#endif // ROBOTEQ_BROADCASTER_HPP_