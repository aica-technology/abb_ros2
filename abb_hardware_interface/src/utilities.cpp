/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * Modifications Copyright (c) 2022, PickNik Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

// This file is a modified copy from
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/src/initialization.cpp
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/src/verification.cpp

#include <abb_hardware_interface/utilities.hpp>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace abb
{
namespace robot
{
namespace utilities
{
namespace
{
/**
 * \brief Max number of attempts when trying to connect to a robot controller via RWS.
 */
constexpr unsigned int RWS_MAX_CONNECTION_ATTEMPTS{ 5 };

/**
 * \brief Error message for failed connection attempts when trying to connect to a robot controller via RWS.
 */
constexpr char RWS_CONNECTION_ERROR_MESSAGE[]{ "Failed to establish RWS connection to the robot controller" };

/**
 * \brief Time [s] to wait before trying to reconnect to a robot controller via RWS.
 */
constexpr uint8_t RWS_RECONNECTION_WAIT_TIME{ 1 };
auto LOGGER = rclcpp::get_logger("ABBHardwareInterfaceUtilities");
}  // namespace

RobotControllerDescription establishRWSConnection(RWSManager& rws_manager, const std::string& robot_controller_id,
                                                  const bool no_connection_timeout)
{
  unsigned int attempt{ 0 };

  while (rclcpp::ok() && (no_connection_timeout || attempt++ < RWS_MAX_CONNECTION_ATTEMPTS))
  {
    try
    {
      return rws_manager.collectAndParseSystemData(robot_controller_id);
    }
    catch (const std::runtime_error& exception)
    {
      if (!no_connection_timeout)
      {
        RCLCPP_WARN_STREAM(LOGGER, RWS_CONNECTION_ERROR_MESSAGE << " (attempt " << attempt << "/"
                                                                << RWS_MAX_CONNECTION_ATTEMPTS << "), reason: '"
                                                                << exception.what() << "'");
      }
      else
      {
        RCLCPP_WARN_STREAM(LOGGER, RWS_CONNECTION_ERROR_MESSAGE << " (waiting indefinitely), reason: '"
                                                                << exception.what() << "'");
      }
      rclcpp::sleep_for(std::chrono::seconds(RWS_RECONNECTION_WAIT_TIME));
    }
  }

  throw std::runtime_error{ RWS_CONNECTION_ERROR_MESSAGE };
}

void verifyRobotWareVersion(const RobotWareVersion& rw_version)
{
  if (rw_version.major_number() == 6 && rw_version.minor_number() < 7 && rw_version.patch_number() < 1)
  {
    auto error_message{ "Unsupported RobotWare version (" + rw_version.name() + ", need at least 6.07.01)" };

    RCLCPP_FATAL_STREAM(LOGGER, error_message);
    throw std::runtime_error{ error_message };
  }
}

bool verifyStateMachineAddInPresence(const SystemIndicators& system_indicators)
{
  return system_indicators.addins().state_machine_1_0() || system_indicators.addins().state_machine_1_1();
}

bool stopRAPIDprogram(RWSManager& rws_manager)
{
  if (!verifyRWSManagerReady(rws_manager))
  {
    return false;
  }

  bool success {}; 
  rws_manager.runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      RCLCPP_INFO_STREAM(LOGGER, "Trying to stop RAPID program in case it is running...");
      interface.stopRAPIDExecution();
      success = true; 
    } 
    catch (...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to stop RAPID program...");
      success = false;
    }
  });
  return success;
}

bool setGPIO(RWSManager& rws_manager, const std::string& signal, bool value)
{
  if (!verifyRWSManagerReady(rws_manager))
  {
    return false;
  }
  if (!verifyArgumentSignal(signal))
  {
    return false;
  }

  bool success {}; 
  rws_manager.runService([&](abb::rws::v2_0::RWSStateMachineInterface& interface) {
    try
    {
      RCLCPP_INFO_STREAM(LOGGER, "Trying to set GPIO...");
      interface.setDigitalSignal(signal, value);
      success = true;
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to stet GPIO...");
      success = false;
    }
  });
  return success;
}


bool verifyRWSManagerReady(RWSManager& rws_manager)
{
  if (!rws_manager.isInterfaceReady())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "RWS Manager not ready...");
    return false;
  }
  return true;
}

bool verifyArgumentSignal(const std::string& signal)
{
  if (signal.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Empty signal...");
    return false;
  }
  return true;
}

}  // namespace utilities
}  // namespace robot
}  // namespace abb
