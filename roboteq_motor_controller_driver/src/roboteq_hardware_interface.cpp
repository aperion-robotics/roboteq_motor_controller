#include <math.h>

#include <roboteq_motor_controller_driver/roboteq_hardware_interface.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("RoboteqHardwareInterface");

namespace roboteq_motor_controller_driver
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

RoboteqHardwareInterface::~RoboteqHardwareInterface()
{
	// If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
	// We therefore need to make sure to actually deactivate the communication
	on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn RoboteqHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) 
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (info_.joints.size() != 2)
	{
		RCLCPP_FATAL(LOGGER, "%zu joints found. 2 were expected.", info_.joints.size());
		return hardware_interface::CallbackReturn::ERROR;
	}

	for (const hardware_interface::ComponentInfo& joint : info_.joints)
	{
		if (joint.command_interfaces.size() != 2) 
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces. 2 expected.", joint.name.c_str(), joint.command_interfaces.size());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) 
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interface as first command interface. '%s' expected.",
						joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) 
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interface as second command interface. '%s' expected.",
						joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.state_interfaces.size() != 2) 
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interfaces. 2 expected.",	joint.name.c_str(), joint.state_interfaces.size());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) 
		{
			RCLCPP_FATAL(LOGGER,"Joint '%s' has %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
						joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) 
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' has %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
						joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
			return hardware_interface::CallbackReturn::ERROR;
		}

		const auto map_iterator = joint.parameters.find("channel");
		if (map_iterator == joint.parameters.end())
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' no channel parameter.", joint.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;	
		}
		else if (map_iterator->second != "1" && map_iterator->second != "2")
		{
			RCLCPP_FATAL(LOGGER, "Joint '%s' has %s as the channel parameter. 1 or 2 expected.", joint.name.c_str(), map_iterator->second.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
	}

	if (info_.joints[0].parameters["channel"] == info_.joints[1].parameters["channel"])
	{
		RCLCPP_FATAL(LOGGER, "Both joints have the same channel parameter. Different channels expected.");
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (info_.transmissions.size() != 2)
	{
		RCLCPP_FATAL(LOGGER, "%zu transmissions found. 2 were expected.", info_.transmissions.size());
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (info_.transmissions[0].joints[0].name == info_.transmissions[1].joints[0].name)
	{
		RCLCPP_FATAL(LOGGER, "Both transmissions have the same joint. Different joints expected.");
		return hardware_interface::CallbackReturn::ERROR;
	}

	// reserve the space needed for vectors
	position_commands_.resize(2);
	velocity_commands_.resize(2);
	joint_positions_.resize(2);
	joint_velocities_.resize(2);
	joints_position_transmission_passthrough_.resize(2);
	joints_velocity_transmission_passthrough_.resize(2);
	actuators_position_transmission_passthrough_.resize(2);
	actuators_velocity_transmission_passthrough_.resize(2);

	// create transmissions, joint and actuator handles
	auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

	for (const auto & transmission_info : info_.transmissions)
	{
		// only simple transmissions are supported
		if (transmission_info.type != "transmission_interface/SimpleTransmission")
		{
			RCLCPP_FATAL(LOGGER, "Transmission '%s' of type '%s' not supported.",
				transmission_info.name.c_str(), transmission_info.type.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (transmission_info.joints.size() != 1)
		{
			RCLCPP_FATAL(LOGGER, "%zu joints found in transmission %s. 1 was expected.", transmission_info.joints.size(), transmission_info.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (transmission_info.actuators.size() != 1)
		{
			RCLCPP_FATAL(LOGGER, "%zu actuators found in transmission %s. 1 was expected.", transmission_info.actuators.size(), transmission_info.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		const auto joint_match = std::find_if(info_.joints.cbegin(), info_.joints.cend(), [&](const auto & joint) { return joint.name == transmission_info.joints[0].name;});

		if(joint_match == info_.joints.cend())
		{
			RCLCPP_FATAL(LOGGER, "No match for joint %s in transmission %s.", transmission_info.joints[0].name.c_str(), transmission_info.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		
		// Convert channel to vector index
		const int index = std::stoi(joint_match->parameters.at("channel")) - 1;

		std::shared_ptr<transmission_interface::Transmission> position_transmission;
		std::shared_ptr<transmission_interface::Transmission> velocity_transmission;
		
		try
		{
			position_transmission = transmission_loader.load(transmission_info);
			velocity_transmission = transmission_loader.load(transmission_info);
		}
		catch (const transmission_interface::TransmissionInterfaceException & exc)
		{
			RCLCPP_FATAL(LOGGER, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
			return hardware_interface::CallbackReturn::ERROR;
		}

		std::vector<transmission_interface::JointHandle> joint_position_handles;
		transmission_interface::JointHandle joint_position_handle(transmission_info.joints[0].name, hardware_interface::HW_IF_POSITION, &(joints_position_transmission_passthrough_[index]));
		joint_position_handles.push_back(joint_position_handle);

		std::vector<transmission_interface::JointHandle> joint_velocity_handles;
		transmission_interface::JointHandle joint_velocity_handle(transmission_info.joints[0].name, hardware_interface::HW_IF_VELOCITY, &(joints_velocity_transmission_passthrough_[index]));
		joint_velocity_handles.push_back(joint_velocity_handle);

		std::vector<transmission_interface::ActuatorHandle> actuator_position_handles;
		transmission_interface::ActuatorHandle actuator_position_handle(transmission_info.actuators[0].name, hardware_interface::HW_IF_POSITION, &(actuators_position_transmission_passthrough_[index]));
		actuator_position_handles.push_back(actuator_position_handle);

		std::vector<transmission_interface::ActuatorHandle> actuator_velocity_handles;
		transmission_interface::ActuatorHandle actuator_velocity_handle(transmission_info.actuators[0].name, hardware_interface::HW_IF_VELOCITY, &(actuators_velocity_transmission_passthrough_[index]));
		actuator_velocity_handles.push_back(actuator_velocity_handle);

		try
		{
			position_transmission->configure(joint_position_handles, actuator_position_handles);
			velocity_transmission->configure(joint_velocity_handles, actuator_velocity_handles);
		}
		catch (const transmission_interface::TransmissionInterfaceException & exc)
		{
			RCLCPP_FATAL(LOGGER, "Error while configuring %s: %s", transmission_info.name.c_str(), exc.what());
			return hardware_interface::CallbackReturn::ERROR;
		}

		position_transmissions_.push_back(position_transmission);
		velocity_transmissions_.push_back(velocity_transmission);
	}

	// Create the parameter listener and get the parameters
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("roboteq_hardware_interface_temp_node");
    std::shared_ptr<ParamListener> param_listener = std::make_shared<ParamListener>(node);
    params_ = param_listener->get_params();

	if (std::find(params_.queries_list.begin(), params_.queries_list.end(), "CR") != params_.queries_list.end())
	{
		position_query_ = "CR";
	}
	else if (std::find(params_.queries_list.begin(), params_.queries_list.end(), "BCR") != params_.queries_list.end())
	{
		position_query_ = "BCR";
	}
	else
	{
		RCLCPP_WARN(LOGGER, "No relative position query found. Defaulting to BCR");
		params_.queries_list.push_back("BCR");
		roboteq_motor_controller_driver::Params::Queries::MapQueriesList bcr_struct;
		bcr_struct.name = "internal_sensor_count_relative";
		bcr_struct.size = 2;
		params_.queries.queries_list_map.insert({"BCR", bcr_struct});
		position_query_ = "BCR";
	}

	if (std::find(params_.queries_list.begin(), params_.queries_list.end(), "S") != params_.queries_list.end())
	{
		velocity_query_ = "S";
	}
	else if (std::find(params_.queries_list.begin(), params_.queries_list.end(), "BS") != params_.queries_list.end())
	{
		velocity_query_ = "BS";
	}
	else
	{
		RCLCPP_WARN(LOGGER, "No speed query found. Defaulting to BS");
		params_.queries_list.push_back("BS");
		roboteq_motor_controller_driver::Params::Queries::MapQueriesList bs_struct;
		bs_struct.name = "internal_sensor_motor_speed";
		bs_struct.size = 2;
		params_.queries.queries_list_map.insert({"BS", bs_struct});
		velocity_query_ = "BS";
	}

	// Check for duplicate names
	std::unordered_set<std::string> name_set;
	for (auto& map_item : params_.queries.queries_list_map)
	{
		if (!name_set.insert(map_item.second.name).second)
		{
			RCLCPP_FATAL(LOGGER, "Found duplicate name (%s) in queries. Each query name should be unique.", map_item.second.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		map_item.second.values.resize(map_item.second.size);
	}

	for (const auto& query : params_.queries_list)
	{
		serial_query_ << "?" << query << "_";
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	for (size_t i = 0; i != joints_position_transmission_passthrough_.size(); ++i)
	{
		joint_positions_[i] = 0.0;
		joints_position_transmission_passthrough_[i] = kNaN;
		joints_velocity_transmission_passthrough_[i] = kNaN;
		actuators_position_transmission_passthrough_[i] = kNaN;
		actuators_velocity_transmission_passthrough_[i] = kNaN;
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	try
	{
		ser_.setPort(params_.serial.port);
		ser_.setBaudrate(params_.serial.baud);
		serial::Timeout to = serial::Timeout::simpleTimeout(100);
		ser_.setTimeout(to);
		ser_.open();
	}
	catch (serial::IOException &e)
	{
		RCLCPP_ERROR(LOGGER, "Unable to open port");
		return hardware_interface::CallbackReturn::ERROR;
	}

	if (ser_.isOpen())
	{
		RCLCPP_INFO(LOGGER, "Serial Port initialized");
		// Disable echo and clear buffer history
		std::stringstream query;
		query << "^echof 1_# c_";
		ser_.write(query.str());
		ser_.flushOutput();
	}
	else
	{
		RCLCPP_ERROR(LOGGER, "Serial Port is not open");
		return hardware_interface::CallbackReturn::ERROR;
	}  

  	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
	if (ser_.isOpen())
	{
		ser_.close();
	} 

  	return hardware_interface::CallbackReturn::SUCCESS;
}

/*
hardware_interface::CallbackReturn RoboteqHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
	if (previous_state.label() == "inactive" || previous_state.label() == "active")
	{
		if(!ser_.isOpen())
		{
			uint attempts = 0;
			do
			{
				RCLCPP_ERROR(LOGGER, "Unable to open serial port. Trying to connect in 5 seconds.");
				rclcpp::sleep_for(std::chrono::seconds(5));
				try
				{
					ser_.setPort(params_.serial.port);
					ser_.setBaudrate(params_.serial.baud);
					serial::Timeout to = serial::Timeout::simpleTimeout(10);
					ser_.setTimeout(to);
					ser_.open();
				}
				catch (serial::IOException &e)
				{
					continue;
				}
			}
			while(!ser_.isOpen() && attempts++ < MAX_RECONNECT_ATTEMPTS);

			if(ser_.isOpen())
			{
				RCLCPP_INFO(LOGGER, "Connection recovered");
				// Close because it will open again in the on_activate transition
				ser_.close();
				return hardware_interface::CallbackReturn::SUCCESS;
			}
			else
			{
				RCLCPP_FATAL(LOGGER, "Could not establish connection.");
			}
		}
	}

  	return hardware_interface::CallbackReturn::FAILURE;
}
*/

std::vector<hardware_interface::StateInterface> RoboteqHardwareInterface::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	for (const auto & joint : info_.joints)
	{
		const int index = std::stoi(joint.parameters.at("channel")) - 1;
		state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint_positions_[index]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[index]));
	}

	for (auto& map_item : params_.queries.queries_list_map)
	{
		for (int i = 0; i < map_item.second.size; ++i)
		{
			state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", map_item.second.name + "_" + std::to_string(i), &map_item.second.values[i]));	
		}
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboteqHardwareInterface::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for (const auto & joint : info_.joints)
	{
		const int index = std::stoi(joint.parameters.at("channel")) - 1;
		command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &position_commands_[index]));
		command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[index]));
	}
	return command_interfaces;
}

hardware_interface::return_type RoboteqHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	if(!ser_.isOpen())
	{
		RCLCPP_ERROR(LOGGER, "Serial Port is not open");
		return hardware_interface::return_type::ERROR;	
	}

	// Request data from roboteq
	ser_.write(serial_query_.str());
	ser_.flushOutput();

	std::vector<std::string> queries_response_list;

	// Read data from serial
	size_t max_line_size{65536};
	std::string eol{"\r"};
	std::string response;
	std::string del;
	size_t del_pos;
	std::string key;
	std::string data;

	int data_start;
	int data_end;
	int i;

	if(!ser_.waitReadable())
	{
		RCLCPP_WARN(LOGGER, "Serial timed out waiting for response");
		return hardware_interface::return_type::OK;
	}

	while (ser_.available())
	{
		response = ser_.readline(max_line_size, eol);

		del = "=";
		del_pos = response.find(del);
		if(del_pos == std::string::npos)
		{	
			continue;
		}

		key = response.substr(0, del_pos);
		data = response.substr(del_pos + del.size());

		if (std::find(params_.queries_list.begin(), params_.queries_list.end(), key) == params_.queries_list.end()) 
		{
			RCLCPP_WARN(LOGGER, "'%s' is not part of the queries list", key.c_str());
			continue;
		}

		queries_response_list.push_back(key);

		del = ":";
		data_end = -1*del.size();
		i = 0;
		do {
			data_start = data_end + del.size();
			data_end = data.find(del, data_start);
			params_.queries.queries_list_map.at(key).values[i] = std::stod(data.substr(data_start, data_end - data_start));
			i = i + 1;
		} while (data_end != -1 and i < params_.queries.queries_list_map.at(key).size);
	}

	if (queries_response_list != params_.queries_list)
	{
		std::string expected;
		std::string actual;
		for(const auto &s : params_.queries_list) expected = expected + s + " ";
		for(const auto &s : queries_response_list) actual = actual + s + " ";
		RCLCPP_WARN(LOGGER, "The queries response did not match the expected one");
		RCLCPP_WARN(LOGGER, "Expected: %s", expected.c_str());
		RCLCPP_WARN(LOGGER, "Got: %s", actual.c_str());
	}

	// actuator: state -> transmission
	for (size_t i = 0; i != actuators_position_transmission_passthrough_.size(); ++i)
	{
		actuators_position_transmission_passthrough_[i] = (params_.queries.queries_list_map.at(position_query_).values[i] / params_.encoder.cpr) * (2*M_PI); // Convert from encoder counts to rad
	}

	for (size_t i = 0; i != actuators_velocity_transmission_passthrough_.size(); ++i)
	{
		actuators_velocity_transmission_passthrough_[i] = (params_.queries.queries_list_map.at(velocity_query_).values[i] * (2*M_PI)) / 60; // Conversion from RPM to 1/s (rad/s)
	}

	// transmission: actuator -> joint
	std::for_each(position_transmissions_.begin(), position_transmissions_.end(), [](auto & transmission) { transmission->actuator_to_joint(); });
	std::for_each(velocity_transmissions_.begin(), velocity_transmissions_.end(), [](auto & transmission) { transmission->actuator_to_joint(); });

	// joint: transmission -> state
	for (size_t i = 0; i != joints_position_transmission_passthrough_.size(); ++i)
	{
		joint_positions_[i] += joints_position_transmission_passthrough_[i] ; // accumulating relative encoder changes to produce absolute position state interface
	}
	for (size_t i = 0; i != joints_velocity_transmission_passthrough_.size(); ++i)
	{
		joint_velocities_[i] = joints_velocity_transmission_passthrough_[i] ;
	}

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboteqHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	if(!ser_.isOpen())
	{
		return hardware_interface::return_type::ERROR;	
	}

	std::stringstream channel_one_cmd;
	std::stringstream channel_two_cmd;

	if (position_controller_running_) 
	{
		// joint: command -> transmission
		for (size_t i = 0; i != joints_position_transmission_passthrough_.size(); ++i)
		{
			joints_position_transmission_passthrough_[i] = position_commands_[i];
		}

		// transmission: joint -> actuator
		std::for_each(position_transmissions_.begin(), position_transmissions_.end(), [](auto & transmission) { transmission->joint_to_actuator(); });

		channel_one_cmd << "!PR 1 " << static_cast<int>(actuators_position_transmission_passthrough_[0]) << "\r";
		channel_two_cmd << "!PR 2 " << static_cast<int>(actuators_position_transmission_passthrough_[1]) << "\r";
    } 
	else if (velocity_controller_running_) 
	{
		// joint: command -> transmission
		for (size_t i = 0; i != joints_velocity_transmission_passthrough_.size(); ++i)
		{
			joints_velocity_transmission_passthrough_[i] = (velocity_commands_[i] / (2*M_PI)) * 60; // Conversion to RPM from 1/s (rad/s)
		}

		// transmission: joint -> actuator
		std::for_each(velocity_transmissions_.begin(), velocity_transmissions_.end(), [](auto & transmission) { transmission->joint_to_actuator(); });

		channel_one_cmd << "!S 1 " << static_cast<int>(actuators_velocity_transmission_passthrough_[0]) << "\r";
		channel_two_cmd << "!S 2 " << static_cast<int>(actuators_velocity_transmission_passthrough_[1]) << "\r";		
	}	

	ser_.write(channel_one_cmd.str());
	ser_.write(channel_two_cmd.str());
	ser_.flush();

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboteqHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
	hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

	start_modes_.clear();
	stop_modes_.clear();

	// Starting interfaces
	// add start interface per joint in tmp var for later check
	for (const auto& key : start_interfaces) 
	{
		for (auto i = 0u; i < info_.joints.size(); i++) 
		{
			if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) 
			{
				start_modes_.push_back(hardware_interface::HW_IF_POSITION);
			}
			if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) 
			{
				start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
			}
		}
	}
	// set new mode to all interfaces at the same time
	if (start_modes_.size() != 0 && start_modes_.size() != 2) 
	{
		ret_val = hardware_interface::return_type::ERROR;
	}

	// all start interfaces must be the same - can't mix position and velocity control
	if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) 
	{
		ret_val = hardware_interface::return_type::ERROR;
	}

	// Stopping interfaces
	// add stop interface per joint in tmp var for later check
	for (const auto& key : stop_interfaces) 
	{
		for (auto i = 0u; i < info_.joints.size(); i++) 
		{
			if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) 
			{
				stop_modes_.push_back(hardware_interface::HW_IF_POSITION);
			}
			if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) 
			{
				stop_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
			}
		}
	}
	// stop all interfaces at the same time
	if (stop_modes_.size() != 0 && stop_modes_.size() != 2) 
	{
		ret_val = hardware_interface::return_type::ERROR;
	}

	// all stop interfaces must be the same - can't mix position and velocity control
	if (stop_modes_.size() != 0 && !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin())) 
	{
		ret_val = hardware_interface::return_type::ERROR;
	}

	return ret_val;
}

hardware_interface::return_type RoboteqHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/)
{
	hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

	if (stop_modes_.size() != 0 && std::find(stop_modes_.begin(), stop_modes_.end(), hardware_interface::HW_IF_POSITION) != stop_modes_.end()) 
	{
		position_controller_running_ = false;
	} 
	else if (stop_modes_.size() != 0 && std::find(stop_modes_.begin(), stop_modes_.end(), hardware_interface::HW_IF_VELOCITY) != stop_modes_.end()) 
	{
		velocity_controller_running_ = false;
		for (auto& command : velocity_commands_) command = 0.0;
	}

	if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) 
	{
		velocity_controller_running_ = false;
		position_controller_running_ = true;

	} 
	else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) 
	{
		position_controller_running_ = false;
		for (auto& command : velocity_commands_) command = 0.0;
		velocity_controller_running_ = true;
	}

	start_modes_.clear();
	stop_modes_.clear();

	return ret_val;
}
} // roboteq_motor_controller_driver namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(roboteq_motor_controller_driver::RoboteqHardwareInterface, hardware_interface::SystemInterface)