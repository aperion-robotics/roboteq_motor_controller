#include <cmath>
#include <map>

#include <roboteq_motor_controller_driver/roboteq_broadcaster.hpp>

namespace roboteq_broadcaster
{

controller_interface::CallbackReturn RoboteqBroadcaster::on_init()
{
	auto_declare<std::string>("hardware_name", "");

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboteqBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	hardware_name_ = get_node()->get_parameter("hardware_name").as_string();
	if (hardware_name_.empty())
	{
		RCLCPP_ERROR(get_node()->get_logger(), "'hardware_name' parameter is empty. Set it to the name of the Roboteq ros2_control hardware component.");
		return controller_interface::CallbackReturn::ERROR;
	}

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboteqBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	// Group the query state interfaces of the hardware component by query name.
	// They are named <hardware_name>/<query_name>_<channel>. The order of ALL interfaces is not
	// guaranteed, so the channels are sorted by their index.
	std::map<std::string, std::map<size_t, size_t>> queries;  // query name -> channel -> index in state_interfaces_
	for (size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		const auto& state_interface = state_interfaces_[i];
		if (state_interface.get_prefix_name() != hardware_name_)
		{
			continue;
		}

		const std::string interface_name = state_interface.get_interface_name();
		const size_t del_pos = interface_name.rfind('_');
		if (del_pos == std::string::npos || del_pos == 0 || del_pos + 1 == interface_name.size() ||
			interface_name.find_first_not_of("0123456789", del_pos + 1) != std::string::npos)
		{
			RCLCPP_DEBUG(get_node()->get_logger(), "Ignoring state interface %s: not a query interface", state_interface.get_name().c_str());
			continue;
		}

		const std::string query_name = interface_name.substr(0, del_pos);
		const size_t channel = std::stoul(interface_name.substr(del_pos + 1));
		queries[query_name][channel] = i;
	}

	if (queries.empty())
	{
		RCLCPP_ERROR(get_node()->get_logger(),
			"No query state interfaces found for hardware component '%s'. Check that 'hardware_name' matches "
			"the name of the Roboteq <ros2_control> tag and that the component is active.", hardware_name_.c_str());
		return CallbackReturn::ERROR;
	}

	for (const auto& [query_name, channels] : queries)
	{
		std::vector<size_t> indices;
		indices.reserve(channels.size());
		for (const auto& [channel, index] : channels)
		{
			indices.push_back(index);
		}

		auto query_publisher = get_node()->create_publisher<roboteq_motor_controller_msgs::msg::Query>(
			hardware_name_ + "/" + query_name, rclcpp::SystemDefaultsQoS());
		auto realtime_query_publisher = std::make_shared<realtime_tools::RealtimePublisher<roboteq_motor_controller_msgs::msg::Query>>(query_publisher);

		// Values stay at 0 until the query is read for the first time
		roboteq_motor_controller_msgs::msg::Query query_msg;
		query_msg.data.resize(indices.size(), 0);

		RCLCPP_INFO(get_node()->get_logger(), "Publishing query '%s' (%zu channels) on %s",
			query_name.c_str(), indices.size(), query_publisher->get_topic_name());

		realtime_roboteq_publishers_.push_back(realtime_query_publisher);
		query_msgs_.push_back(query_msg);
		query_state_interface_indices_.push_back(std::move(indices));
	}

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboteqBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	for (auto& publisher : realtime_roboteq_publishers_)
	{
		publisher.reset();
	}
	realtime_roboteq_publishers_.clear();
	query_msgs_.clear();
	query_state_interface_indices_.clear();

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RoboteqBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
	for (size_t i = 0; i < realtime_roboteq_publishers_.size(); i++)
	{
		auto & query_msg = query_msgs_[i];
		query_msg.header.stamp = time;
		const auto & indices = query_state_interface_indices_[i];
		for (size_t j = 0; j < indices.size(); j++)
		{
			// keep the previous value if the state interface could not be read this cycle
			const auto value = state_interfaces_[indices[j]].get_optional();
			if (value.has_value() && !std::isnan(*value))
			{
				query_msg.data[j] = static_cast<int64_t>(*value);
			}
		}

		realtime_roboteq_publishers_[i]->try_publish(query_msg);
	}

	return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration RoboteqBroadcaster::command_interface_configuration() const
{
	return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration RoboteqBroadcaster::state_interface_configuration() const
{
	// Claim all state interfaces (read-only, so they can be shared with other controllers)
	// and keep the ones of the hardware component in on_activate
	return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::ALL};
}


} // roboteq_broadcaster namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(roboteq_broadcaster::RoboteqBroadcaster, controller_interface::ControllerInterface)
