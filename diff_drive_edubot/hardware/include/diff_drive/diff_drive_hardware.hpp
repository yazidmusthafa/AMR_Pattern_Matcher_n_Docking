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

#ifndef DIFF_DRIVE__HARDWARE_HPP_
#define DIFF_DRIVE__HARDWARE_HPP_

#include "string"
#include <memory>
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diff_drive/visibility_control.h"
#include "diff_drive/arduino_comms.hpp"

using hardware_interface::return_type;

namespace diff_drive
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{

struct Config
{
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem);

  DIFF_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DIFF_DRIVE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFF_DRIVE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFF_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  DIFF_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFF_DRIVE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFF_DRIVE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  ArduinoComms comms_;
  Config cfg_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  // namespace ros2_control_demo_example_7

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_HARDWARE_HPP_
