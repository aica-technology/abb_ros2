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
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/include/abb_robot_cpp_utilities/initialization.h
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/include/abb_robot_cpp_utilities/verification.h

#pragma once

#include <string>

#include <abb_egm_rws_managers/rws_manager.h>
#include <string>

namespace abb
{
namespace robot
{
namespace utilities
{
/**
 * \brief Attempts to establish a connection to a robot controller's RWS server.
 *
 * If a connection is established, then a structured description of the robot controller is returned.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 * \param robot_controller_id for an identifier/nickname for the targeted robot controller.
 * \param no_connection_timeout indicator whether to wait indefinitely on the robot controller.
 *
 * \return RobotControllerDescription of the robot controller.
 *
 * \throw std::runtime_error if unable to establish a connection.
 */
RobotControllerDescription establishRWSConnection(RWSManager& rws_manager, const std::string& robot_controller_id,
                                                  const bool no_connection_timeout);

/**
 * \brief Verifies that the RobotWare version is supported.
 *
 * Note: For now, only RobotWare versions in the range [6.07.01, 7.0) are supported (i.e. excluding 7.0).
 *
 * \param rw_version to verify.
 *
 * \throw std::runtime_error if the RobotWare version is not supported.
 */
void verifyRobotWareVersion(const RobotWareVersion& rw_version);

/**
 * \brief Verifies that the RobotWare StateMachine Add-In is present in a system.
 *
 * \param system_indicators to verify.
 *
 * \return bool true if the StateMachine Add-In is present.
 */
bool verifyStateMachineAddInPresence(const SystemIndicators& system_indicators);

/**
 * \brief Starts the RAPID program that is running.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 *
 */
bool startRAPIDprogram(RWSManager& rws_manager);

/**
 * \brief Stops the RAPID program that is running.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 *
 */
bool stopRAPIDprogram(RWSManager& rws_manager);

/**
 * \brief Reads a file from a path and uploads it in the controller Home directory.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 * \param source_file_path the RAPID file path to read and upload.
 * \param target_file_name the desired name in the controller directory.
 *
 */
bool uploadFile(RWSManager& rws_manager, const std::string& source_file_path, const std::string& target_file_name);

/**
 * \brief Loads a module into the controller task.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 * \param task_name the desired task name.
 * \param module_name the desired name in the controller directory.
 *
 */
bool loadModuleIntoTask(RWSManager& rws_manager, const std::string& task_name, const std::string& module_name);

/**
 * \brief Sets a GPIO.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 * \param signal the signal name.
 * \param value the value to be written.
 *
 */
bool setGPIO(RWSManager& rws_manager, const std::string& signal, bool value);

/**
 * \brief Starts the motors.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 *
 */
bool startMotors(RWSManager& rws_manager);

/**
 * \brief Stops the motors.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 *
 */
bool stopMotors(RWSManager& rws_manager);

/**
 * \brief Resets the program pointer to main.
 *
 * \param rws_manager for handling the RWS communication with the robot controller.
 *
 */
bool resetPP(RWSManager& rws_manager);

/**
 * \brief Verifys that an argument is not empty.
 *
 * \param signal the signal name.
 *
 */
bool verifyArgument(const std::string& signal);

/**
 * \brief Verifys that the RWS manager is ready.
 *
 * \param rws_manager to check.
 *
 */
bool verifyRWSManagerReady(RWSManager& rws_manager);

}  // namespace utilities
}  // namespace robot
}  // namespace abb
