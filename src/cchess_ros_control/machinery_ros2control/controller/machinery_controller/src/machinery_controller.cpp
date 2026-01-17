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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "machinery_controller/machinery_controller.hpp"
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

// controller 是搭配 yaml文件 开发的
namespace machinery_controller
{
MachineryController::MachineryController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MachineryController::on_init()
{
  // 自动声明参数 "cartesian_joint_name"
  auto_declare<std::string>("cartesian_joint_name", "");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MachineryController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 获取参数
  cartesian_joint_name_ = get_node()->get_parameter("cartesian_joint_name").as_string();
  if (cartesian_joint_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'cartesian_joint_name'参数没有设置");
    return CallbackReturn::ERROR;
  }

  // 创建 Pub 和 Sub
  command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { received_command_ptr_.set(msg); });
  task_status_publisher_ = get_node()->create_publisher<machinery_control_msg::msg::TaskStatus>("~/task_status",10);

  // 初始化变量
  last_command_interfaces_ = std::vector<double>(3);
  task_status_.header.frame_id = this->get_node()->get_name();
  task_status_.is_idle = true;
  epsilon = 1e-6;

  RCLCPP_INFO(get_node()->get_logger(), "配置成功");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MachineryController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(cartesian_joint_name_ + "/x_position");
  command_interfaces_config.names.push_back(cartesian_joint_name_ + "/y_position");
  command_interfaces_config.names.push_back(cartesian_joint_name_ + "/z_position");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MachineryController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.push_back(cartesian_joint_name_ + "/x_position");
  state_interfaces_config.names.push_back(cartesian_joint_name_ + "/y_position");
  state_interfaces_config.names.push_back(cartesian_joint_name_ + "/z_position");

  return state_interfaces_config;
}

controller_interface::CallbackReturn MachineryController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 在激活时，将命令指针重置为nullptr
  received_command_ptr_.set(nullptr);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MachineryController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MachineryController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 接收 业务代码 -> controller的数据，即要发送到串口的目标点
  std::shared_ptr<CmdType> command;
  received_command_ptr_.get(command);
  if (!command) return controller_interface::return_type::OK; // 如果指针为空，说明还未收到任何命令

  // 接收 hardware_interface -> controller的数据，即串口返回的已到达的点
  if (std::abs(last_command_interfaces_[0] - state_interfaces_[0].get_value()) > epsilon ||
      std::abs(last_command_interfaces_[1] - state_interfaces_[1].get_value()) > epsilon ||
      std::abs(last_command_interfaces_[2] - state_interfaces_[2].get_value()) > epsilon)
    task_status_.is_idle = false;
  else
    task_status_.is_idle = true;
  task_status_.header.stamp = this->get_node()->now();
  task_status_publisher_->publish(task_status_);

  // 将命令写入硬件接口
  // command_interfaces_ 的顺序与 command_interface_configuration() 中定义的顺序一致
  command_interfaces_[0].set_value(command->point.x);
  command_interfaces_[1].set_value(command->point.y);
  command_interfaces_[2].set_value(command->point.z);
  // 更新last_command_interfaces_的值
  for (int i =0;i<3;i++)
    last_command_interfaces_[i] = command_interfaces_[i].get_value();

  return controller_interface::return_type::OK;
}

}  // namespace machinery_controller

PLUGINLIB_EXPORT_CLASS(
  machinery_controller::MachineryController, controller_interface::ControllerInterface)
