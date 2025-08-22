// Copyright 2020 ROS2-Control Development Team
// Modifications Copyright 2022 PickNik Inc
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

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <abb_hardware_interface/utilities.hpp>

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
static constexpr size_t NUM_CONNECTION_TRIES = 100;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemHardware");

CallbackReturn ABBSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Validate interfaces configured in ros2_control xacro.
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // By default, construct the robot_controller_description_ by connecting to RWS.
  // If configure_via_rws is set to false, configure the robot_controller_description_
  // relying on joint information in the ros2_control xacro.
  const auto configure_it = info_.hardware_parameters.find("configure_via_rws");
  const bool configure_via_rws = configure_it == info_.hardware_parameters.end()                    ? true :
                                 configure_it->second == "false" || configure_it->second == "False" ? false :
                                                                                                      true;
  const auto rws_port = stoi(info_.hardware_parameters["rws_port"]);
  const auto rws_ip = info_.hardware_parameters["rws_ip"];
  const auto rapid_file_path = info_.hardware_parameters["rapid_file_path"];

  rws_manager_ = std::make_unique<abb::robot::RWSManager>(rws_ip, rws_port, "Default User", "robotics");
            
  if (configure_via_rws)
  {
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from RWS.");

    if (rws_ip == "None")
    {
      RCLCPP_FATAL(LOGGER, "RWS IP not specified");
      return CallbackReturn::ERROR;
    }

    // Get robot controller description from RWS
    robot_controller_description_ = abb::robot::utilities::establishRWSConnection(*rws_manager_, "IRB1200", true);
  }
  else
  {
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from HardwareInfo.");

    // Add header.
    auto header{ robot_controller_description_.mutable_header() };
    // Omnicore controllers have RobotWare version >=7.0.0.
    header->mutable_robot_ware_version()->set_major_number(7);
    header->mutable_robot_ware_version()->set_minor_number(3);
    header->mutable_robot_ware_version()->set_patch_number(2);

    // Add system indicators.
    auto system_indicators{ robot_controller_description_.mutable_system_indicators() };
    system_indicators->mutable_options()->set_egm(true);

    // Add single mechanical units group.
    auto mug{ robot_controller_description_.add_mechanical_units_groups() };
    mug->set_name("");

    // Add single robot to mechanical units group.
    auto robot{ mug->mutable_robot() };
    robot->set_type(abb::robot::MechanicalUnit_Type_TCP_ROBOT);
    robot->set_axes_total(info_.joints.size());
    robot->set_mode(abb::robot::MechanicalUnit_Mode_ACTIVATED);

    // Add joints to robot.
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const hardware_interface::ComponentInfo& joint = info_.joints[i];
      // We assume it's a revolute joint unless explicitly specified.
      // Check if a "type" key is present in joint.parameters with value other than "revolute"
      // as per sdformat conventions http://sdformat.org/spec?elem=joint.
      const auto type_it = joint.parameters.find("type");
      const bool is_revolute = type_it != joint.parameters.end() && type_it->second != "revolute" ? false : true;

      // Get the range of the joint from its command interfaces.
      for (const hardware_interface::InterfaceInfo& joint_info : joint.command_interfaces)
      {
        if (joint_info.name == hardware_interface::HW_IF_POSITION)
        {
          const double min = std::stod(joint_info.min);
          const double max = std::stod(joint_info.max);

          abb::robot::StandardizedJoint* p_joint = robot->add_standardized_joints();
          p_joint->set_standardized_name(joint.name);
          p_joint->set_rotating_move(is_revolute);
          p_joint->set_lower_joint_bound(min);
          p_joint->set_upper_joint_bound(max);

          RCLCPP_INFO(LOGGER, "Configured component %s of type %s with range [%.3f, %.3f]", joint.name.c_str(),
                      joint.type.c_str(), min, max);
          break;
        }
      }
    }
  }

   if (!abb::robot::utilities::stopRAPIDprogram(*rws_manager_))
   {
    return CallbackReturn::ERROR;
   }

  rws_manager_->runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      FILE* file = std::fopen(rapid_file_path.c_str(), "rb");  
      if (!file)
      {
        throw std::runtime_error(std::string("Failed to open file: ") + rapid_file_path);
      }

      std::fseek(file, 0, SEEK_END);
      long size = std::ftell(file);
      std::rewind(file);

      std::string buffer;
      buffer.resize(size);

      if (size > 0)
      {
        size_t read = std::fread(&buffer[0], 1, size, file);
        buffer.resize(read);
      }

      std::fclose(file);

      RCLCPP_INFO_STREAM(LOGGER, "Trying to upload file to controller...");
      interface.uploadFile(abb::rws::FileResource("main.mod"), buffer);
    } 
    catch (...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to upload file...");
      // return CallbackReturn::ERROR;
    }
  });

  rclcpp::sleep_for(1000ms);

  rws_manager_->runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      RCLCPP_INFO_STREAM(LOGGER, "Trying to load module to task...");
      interface.loadModuleIntoTask("T_ROB1", abb::rws::FileResource("main.mod"), true);
    } 
    catch (...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to load module...");
      // return CallbackReturn::ERROR;
    }
  });

  rclcpp::sleep_for(1000ms);
  
    rws_manager_->runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      RCLCPP_INFO_STREAM(LOGGER, "Trying to set pp to main.....");
      interface.resetRAPIDProgramPointer();
      RCLCPP_WARN_STREAM(LOGGER, "pp set to main.....");
    }
    catch(...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to reset pointer...");
    }
  });

  rclcpp::sleep_for(1000ms);

  rws_manager_->runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      RCLCPP_WARN_STREAM(LOGGER, "Trying to start motors.....");
      interface.setMotorsOn();
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to start motors...");
    }
  });

  rclcpp::sleep_for(1000ms);

  rws_manager_->runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      RCLCPP_INFO_STREAM(LOGGER, "Trying to start RAPID program...");
      interface.startRAPIDExecution();
    } 
    catch (...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to start RAPID program...");
      // return CallbackReturn::ERROR;
    }
  });

  RCLCPP_INFO_STREAM(LOGGER, "Robot controller description:\n"
                                 << abb::robot::summaryText(robot_controller_description_));

  // Configure EGM
  RCLCPP_INFO(LOGGER, "Configuring EGM interface...");

  // Initialize motion data from robot controller description
  try
  {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description_);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  // Create channel configuration for each mechanical unit group
  std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
  for (const auto& group : robot_controller_description_.mechanical_units_groups())
  {
    try
    {
      const auto egm_port = stoi(info_.hardware_parameters[group.name() + "egm_port"]);
      const auto channel_configuration =
          abb::robot::EGMManager::ChannelConfiguration{ static_cast<uint16_t>(egm_port), group };
      channel_configurations.emplace_back(channel_configuration);
      RCLCPP_INFO_STREAM(LOGGER,
                         "Configuring EGM for mechanical unit group " << group.name() << " on port " << egm_port);
    }
    catch (std::invalid_argument& e)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "EGM port for mechanical unit group \"" << group.name()
                                                                          << "\" not specified in hardware parameters");
      return CallbackReturn::ERROR;
    }
  }
  try
  {
    egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
    return CallbackReturn::ERROR;
  }

  // initialize variables
  first_pass_ = true;
  initialized_ = false;
  async_thread_shutdown_ = false;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ABBSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        const auto pos = joint.name.find("joint");
        const auto joint_name = joint.name.substr(pos);
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        const auto pos = joint.name.find("joint");
        const auto joint_name = joint.name.substr(pos);
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.command.position));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity));
      }
    }
  }

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "abb_stop_RAPID_program", "abb_stop_RAPID_program_cmd", &abb_stop_RAPID_program_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "abb_stop_RAPID_program", "abb_stop_RAPID_program_success", &abb_stop_RAPID_program_success_));

  return command_interfaces;
}

CallbackReturn ABBSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  size_t counter = 0;
  RCLCPP_INFO(LOGGER, "Connecting to robot...");
  while (rclcpp::ok() && counter++ < NUM_CONNECTION_TRIES)
  {
    // Wait for a message on any of the configured EGM channels.
    if (egm_manager_->waitForMessage(500))
    {
      RCLCPP_INFO(LOGGER, "Connected to robot");
      break;
    }

    RCLCPP_INFO(LOGGER, "Not connected to robot...");
    if (counter == NUM_CONNECTION_TRIES)
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
      return CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(500ms);
  }

  egm_manager_->read(motion_data_);
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        joint.command.position = joint.state.position;
        joint.command.velocity = 0.0;
      }
    }
  }

  async_thread_ = std::make_shared<std::thread>(&ABBSystemHardware::asyncThread, this);

  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

  return CallbackReturn::SUCCESS;
}

return_type ABBSystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  egm_manager_->read(motion_data_);
  
  if (first_pass_ && !initialized_) {
    initAsyncIO();
    initialized_ = true;
    first_pass_ = false;
  }

  return return_type::OK;
}

return_type ABBSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  egm_manager_->write(motion_data_);
  return return_type::OK;
}

void ABBSystemHardware::asyncThread()
{
  while (!async_thread_shutdown_) {
    if (initialized_) {
      checkAsyncIO();
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
  }
}

void ABBSystemHardware::checkAsyncIO()
{
  // if (!rtde_comm_has_been_started_) {
  //   return;
  // }

  if (!std::isnan(start_rapid_cmd_) && rws_manager_ != nullptr) {
    try {
      RCLCPP_INFO(LOGGER, "Stopping the RAPID program...");
      start_rapid_async_success_ = rws_manager_->sendRobotProgram();
    }
    catch (...) {
      RCLCPP_ERROR(LOGGER, "Stopping the RAPID program failed...");
    }
    start_rapid_cmd_ = NO_NEW_CMD_;
  }
}

void ABBSystemHardware::initAsyncIO()
{
  start_rapid_cmd_ = NO_NEW_CMD_;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemHardware, hardware_interface::SystemInterface)
