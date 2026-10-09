#include <roboteq_motor_controller_driver/roboteq_broadcaster.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("RoboteqBroadcaster");

namespace roboteq_broadcaster
{

constexpr int kNaN = std::numeric_limits<int>::quiet_NaN();


controller_interface::CallbackReturn RoboteqBroadcaster::on_init()
{
	
	param_listener_ = std::make_shared<roboteq_motor_controller_driver::ParamListener>(get_node());
	params_ = param_listener_->get_params();

	const bool found_cr{std::find(params_.queries_list.begin(), params_.queries_list.end(), "CR") != params_.queries_list.end()};
	const bool found_bcr{std::find(params_.queries_list.begin(), params_.queries_list.end(), "BCR") != params_.queries_list.end()};

	if (!found_cr && !found_bcr)
	{
		params_.queries_list.push_back("BCR");
		roboteq_motor_controller_driver::Params::Queries::MapQueriesList bcr_struct;
		bcr_struct.name = "internal_sensor_count_relative";
		bcr_struct.size = 2;
		params_.queries.queries_list_map.insert({"BCR", bcr_struct});
	}
	
	const bool found_s{std::find(params_.queries_list.begin(), params_.queries_list.end(), "S") != params_.queries_list.end()};
	const bool found_bs{std::find(params_.queries_list.begin(), params_.queries_list.end(), "BS") != params_.queries_list.end()};

	if (!found_s && !found_bs)
	{
		params_.queries_list.push_back("BS");
		roboteq_motor_controller_driver::Params::Queries::MapQueriesList bs_struct;
		bs_struct.name = "internal_sensor_motor_speed";
		bs_struct.size = 2;
		params_.queries.queries_list_map.insert({"BS", bs_struct});
	}

	// Check for duplicate names
	std::unordered_set<std::string> name_set;
	for (auto& map_item : params_.queries.queries_list_map)
	{
		if (!name_set.insert(map_item.second.name).second)
		{
			RCLCPP_FATAL(LOGGER, "Found duplicate name (%s) in queries. Each query name should be unique.", map_item.second.name.c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
	}
	
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboteqBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboteqBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	if (state_interfaces_.empty())
	{
		RCLCPP_ERROR(LOGGER, "None of requested interfaces exist. Broadcaster will not run.");
    	return CallbackReturn::ERROR;
	}
	
	int requested_interfaces_number{0};
	for (const auto& map_item : params_.queries.queries_list_map)
	{
		requested_interfaces_number += map_item.second.size;
	}

	if (state_interfaces_.size() != static_cast<size_t>(requested_interfaces_number))
	{
		RCLCPP_WARN(LOGGER,	"Not all requested interfaces exists. Check ControllerManager output for more detailed information.");
	}
	
	int state_interface_counter = 0;
	std::vector<std::string> queries_visited;
	for (const auto& state_interface : state_interfaces_)
	{
		std::string interface_name = state_interface.get_interface_name();
		std::string del{"_"};
		size_t del_pos = interface_name.rfind(del);
		if(del_pos == std::string::npos)
		{
			RCLCPP_ERROR(LOGGER, "Interface name %s without '%s' delimiter", interface_name.c_str(), del.c_str());
			return CallbackReturn::ERROR;
		}

		std::string query_name = interface_name.substr(0, del_pos);
		if (std::find(queries_visited.cbegin(), queries_visited.cend(), query_name) == queries_visited.cend())
		{	
			cumulative_state_interface_index.push_back(state_interface_counter);
			queries_visited.push_back(query_name);
		}
		state_interface_counter++;
	}
	cumulative_state_interface_index.push_back(state_interface_counter);

	for (size_t i = 0; i < queries_visited.size(); i++)
	{
		int size_found = cumulative_state_interface_index[i+1] - cumulative_state_interface_index[i];
		// Check if number of state interfaces matches what we expect from that particular query
		bool found{false};
		for (const auto& map_item : params_.queries.queries_list_map)
		{
			if (map_item.second.name == queries_visited[i])
			{
				found = true;
				if (size_found != map_item.second.size)
				{
					RCLCPP_ERROR(LOGGER, "Query %s has %d state interfaces associated with it. Expected %ld.", queries_visited[i].c_str(), size_found, map_item.second.size);
					return CallbackReturn::ERROR;
				}
			}
		}
		
		if (!found)
		{
			RCLCPP_ERROR(LOGGER, "Query %s was not found in map", queries_visited[i].c_str());
			return CallbackReturn::ERROR;
		}

		auto query_publisher = get_node()->create_publisher<roboteq_motor_controller_msgs::msg::Query>("roboteq/" + queries_visited[i], rclcpp::SystemDefaultsQoS());
		auto realtime_query_publisher = std::make_shared<realtime_tools::RealtimePublisher<roboteq_motor_controller_msgs::msg::Query>>(query_publisher);

		// default initialization for query message
		auto & query_msg = realtime_query_publisher->msg_;
		query_msg.data.resize(size_found, kNaN);

		realtime_roboteq_publishers_.push_back(realtime_query_publisher);
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

	cumulative_state_interface_index.clear();

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RoboteqBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
	for (size_t i = 0; i < realtime_roboteq_publishers_.size(); i++)
	{
		if (realtime_roboteq_publishers_[i]->trylock())
		{
			auto & query_msg = realtime_roboteq_publishers_[i]->msg_;
			query_msg.header.stamp = time;
			for (int j = cumulative_state_interface_index[i]; j < cumulative_state_interface_index[i+1]; j++)
			{
				query_msg.data[j - cumulative_state_interface_index[i]] = static_cast<int64_t>(state_interfaces_[j].get_value());
			}

			realtime_roboteq_publishers_[i]->unlockAndPublish();
		}
	}

	return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration RoboteqBroadcaster::command_interface_configuration() const
{
  	return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration RoboteqBroadcaster::state_interface_configuration()  const
{

 	controller_interface::InterfaceConfiguration state_interfaces_config;
	state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	for (auto& map_item : params_.queries.queries_list_map)
	{
		for (int i = 0; i < map_item.second.size; ++i)
		{
			state_interfaces_config.names.push_back("gpio/" + map_item.second.name + "_" + std::to_string(i));	
		}
	}	

  	return state_interfaces_config;
}


} // roboteq_broadcaster namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(roboteq_broadcaster::RoboteqBroadcaster, controller_interface::ControllerInterface)