// Copyright 2023 ros2_control Development Team
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

#include "diff_drive/diff_drive_hardware.hpp"
#include <string>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diff_drive
{

  bool use_sim;

hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 4 joints and 2 interfaces
  joint_position_.assign(4, 0);
  joint_velocities_.assign(4, 0);
  joint_position_command_.assign(4, 0);

  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  const char* use_sim_env = std::getenv("USE_SIM");
  use_sim = use_sim_env != nullptr && std::string(use_sim_env) == "true";

  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "USE_SIM ******** %s *****", use_sim ? "true" : "false");

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RobotSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (use_sim == false){ 

    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully configured!");

  }
  else{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Robotic System started as Simulation.......!!");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (use_sim == false){ 
      
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully cleaned up!");

  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RobotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Activating ...please wait...");
  if (use_sim == false){ 
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber
  if (use_sim == false)
  { 
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    comms_.read_servo_values(joint_position_, joint_position_command_);
  }   
  else{
    for (auto i = 0ul; i < joint_position_command_.size(); i++)
    {
      joint_position_[i] = joint_position_command_[i];
    }
  }

  // RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "joint_position_command_: %f, joint_position_: %f",joint_position_[2]*180/3.14159265359, joint_position_command_[2]*180/3.14159265359);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if(use_sim == false)
  {
    if (!comms_.connected())
      {
        return hardware_interface::return_type::ERROR;
      }
  }
  // comms_.set_motor_values(joint_position_command_); 
  // comms_.read_servo_values(joint_position_);   

  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diff_drive::RobotSystem, hardware_interface::SystemInterface)
