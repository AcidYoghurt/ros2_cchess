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

#include "cartesian_move_controller/cartesian_move_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility
using ControllerReferenceMsg = cartesian_move_controller::CartesianMoveController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, size_t interface_count)
{
  msg->displacements.assign(interface_count, std::numeric_limits<double>::quiet_NaN());
  msg->velocities.assign(interface_count, std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace cartesian_move_controller
{
CartesianMoveController::CartesianMoveController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn CartesianMoveController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<cartesian_move_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianMoveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 如果用户单独配置了 state_joints，就用它们作为 state interface 的 joint；
  // 否则默认使用 command interface 中配置的 joints。
  params_ = param_listener_->get_params();
  if (!params_.state_joints.empty())
  {
    // params_.state_joints 是 state_interface 的 joint
    state_joints_ = params_.state_joints;
  }
  else
  {
    // params_.joints 是 command_interface 的 joint
    state_joints_ = params_.joints;
  }

  // 校验：params_.joints的数量 要跟 state_joints_的数量对应
  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(get_node()->get_logger(),"YAML参数中joints的数量(有%zu个)与controller中joints的数量(有%zu个)不匹配！joint的数量与interface的数量相等",params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // 话题 Qos
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // 接收者，用于接收 上层软件 -> controller 的控制指令
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&CartesianMoveController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.interfaces.size());
  input_ref_.writeFromNonRT(msg);

  // 发布者
  try
  {
    // 发布者，用于发布 机械臂此时的状态（忙碌 or 空闲）
    s_publisher_ =get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

    // TODO(anyone): Reserve memory in state publisher depending on the message type
    // 为realtimepublishr预留空间
    state_publisher_->lock();
    state_publisher_->msg_.header.frame_id = get_node()->get_name();
    state_publisher_->unlock();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "在配置阶段创建 publisher 时抛出异常，消息如下 : %s \n",e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "配置成功");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ref_subscriber_的回调函数，只负责更新 input_ref_
void CartesianMoveController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size())
    // 把接收到的 上位软件 的指令存放到 input_ref_中
    input_ref_.writeFromNonRT(msg); // input_ref_中的指令 在 update中被取出并执行
  else
    RCLCPP_ERROR(get_node()->get_logger(),"收到的命令中包含 %zu 个joint，但期望的是 %zu 个joint。忽略该消息。",msg->joint_names.size(), params_.joints.size());
}

// 配置 command_interface
controller_interface::InterfaceConfiguration CartesianMoveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
    for (size_t i = 0;i<params_.interfaces.size();i++)
      command_interfaces_config.names.push_back(joint + "/" + params_.interfaces[i]);

  return command_interfaces_config;
}

// 配置 state_interface
controller_interface::InterfaceConfiguration CartesianMoveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
    for (size_t i = 0;i<params_.interfaces.size();i++)
      state_interfaces_config.names.push_back(joint + "/" + params_.interfaces[i]);

  return state_interfaces_config;
}

controller_interface::CallbackReturn CartesianMoveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // 在激活时，将命令指针重置为nullptr
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.interfaces.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianMoveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停用 controller 时，清空所有 command_interface
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMoveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // controller -> hardware_interface（发送控制指令）
  // 从 input_ref_ 中取出指令(这里用current_ref作为中间变量)，并赋予给 command_interfaces_

  auto current_ref = input_ref_.readFromRT();
  // RCLCPP_INFO(get_node()->get_logger(),"接收到目标点位：%f,%f,%f",(*current_ref)->displacements[0],(*current_ref)->displacements[1],(*current_ref)->displacements[2]);
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      command_interfaces_[i].set_value((*current_ref)->displacements[i]);

      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  // controller -> 上位软件（发送机器人状态）
  // 发布当前机器人状态（空闲 or 忙碌）
  if (state_publisher_ && state_publisher_->trylock())
  {
    // 校验
    if (command_interfaces_.size() != state_interfaces_.size())
    {
      fprintf(stderr, "command_interfaces的数量与state_interfaces的数量不相等！\n");
      return controller_interface::return_type::ERROR;
    }

    // 校验成功
    size_t is_idle = 1;
    for (size_t i =0;i<command_interfaces_.size();i++)
    {
      if (abs(command_interfaces_[i].get_value() - state_interfaces_[i].get_value()) > 1e-6)
        is_idle = 0;
    }
    state_publisher_->msg_.set_point = is_idle;
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace cartesian_move_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_move_controller::CartesianMoveController, controller_interface::ControllerInterface)
