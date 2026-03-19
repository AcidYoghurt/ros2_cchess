// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef MACHINERY_HARDWARE_INTERFACE__MACHINERY_HARDWARE_INTERFACE_HPP_
#define MACHINERY_HARDWARE_INTERFACE__MACHINERY_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <rclcpp/logger.hpp>
#include <memory>
#include <libserial/SerialPort.h>

#include "machinery_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace machinery_hardware_interface
{
class MachineryHardwareInterface : public hardware_interface::SystemInterface
{
public:
  MachineryHardwareInterface(); //默认构造函数，没有实际作用，只是为了防止logger报错

  bool restart_serial(); //重启串口函数

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 使用独立的变量来存储笛卡尔坐标的状态和指令
  // 控制器将直接读写这些变量
  // 笛卡尔积坐标
  double hw_command_x_ = 0.0;
  double hw_command_y_ = 0.0;
  double hw_command_z_ = 0.0;
  double last_hw_command_x_ = 0.0;
  double last_hw_command_y_ = 0.0;
  double last_hw_command_z_ = 0.0;
  double hw_state_x_ = 0.0;
  double hw_state_y_ = 0.0;
  double hw_state_z_ = 0.0;
  // 吸嘴
  double hw_command_suction_ = 0.0;
  double hw_state_suction_ = 0.0;
  double last_hw_command_suction_ = 0.0;

  std::unique_ptr<LibSerial::SerialPort> serial_port_;  // 串口
  rclcpp::Logger logger_; //用于发布消息

  // read函数中的变量
  std::string response;

  // 从URDF中获取的参数
  std::vector<double> origin_position;
  std::vector<double> custom_origin_position;
  std::string frame_prefix_;
  std::string serial_port_name_;
  LibSerial::BaudRate baud_rate_;
};

}  // namespace machinery_hardware_interface

#endif  // MACHINERY_HARDWARE_INTERFACE__MACHINERY_HARDWARE_INTERFACE_HPP_
