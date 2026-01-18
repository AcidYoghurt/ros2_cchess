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

#include "gripper_suction_controller/gripper_suction_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace // 存放工具函数
{
using ControllerReferenceMsg = gripper_suction_controller::GripperSuctionController::ControllerReferenceMsg;

// 构造一条“语义上是『没有指令』，但结构上是『完全合法』的 reference 消息
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace gripper_suction_controller
{
GripperSuctionController::GripperSuctionController() : controller_interface::ControllerInterface() {}

// 初始化
controller_interface::CallbackReturn GripperSuctionController::on_init()
{
  // 初始化参数监听器
  try
  {
    param_listener_ = std::make_shared<gripper_suction_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "调用controller中的on_init函数失败！具体消息: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// 配置
controller_interface::CallbackReturn GripperSuctionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 1. 读取参数
  params_ = param_listener_->get_params();

  // 2. 统一 joints 和 state_joints 的数量
  // params_.joints：控制（Command）用的关节列表（我们要去驱动的关节）
  // params_.state_joints：反馈（State）用的关节列表（我们要读取数据的关节）
  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }
  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(get_node()->get_logger(),"GripperSuctionController 只支持 %zu 个 joint（吸嘴），但配置了 %zu 个",params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // 话题方面配置
  // Qos策略
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // 3. 指令订阅者，接收 上层 -> controller 的控制指令
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>("~/reference", subscribers_qos,std::bind(&GripperSuctionController::reference_callback, this, std::placeholders::_1));

  // 4. 给控制器一个初始 / 安全的 reference，相当于上电默认指令。
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  // 5. 发布者（RealtimePublisher），用来向外发布“控制器当前状态”，即 controller -> 上位软件
  try
  {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);  // 把 发布者s_publisher_ 包装为一个 RealtimePublisher
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "在配置阶段创建 RealtimePublisher 时抛出异常，消息如下: %s \n",e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // 配置 RealtimePublisher
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0]; // msg_ 是 RealtimePublisher 内部预分配的一条“可写消息缓冲区”
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "配置成功");
  return controller_interface::CallbackReturn::SUCCESS;
}

// 定义控制器需要使用的“命令接口”（Command Interfaces）
controller_interface::InterfaceConfiguration GripperSuctionController::command_interface_configuration() const
{
  // 定义配置对象，用于存储配置信息并最终返回给管理器。
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 构建接口名称列表
  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

// 定义控制器需要使用的“状态接口”（State Interfaces）
controller_interface::InterfaceConfiguration GripperSuctionController::state_interface_configuration() const
{
  // 声明配置
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 构建接口名称列表
  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

// 激活，强制把当前 reference 重置成“无指令 / 安全态，进入控制状态
controller_interface::CallbackReturn GripperSuctionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 把当前 reference 重置成默认值
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  return controller_interface::CallbackReturn::SUCCESS;
}

// 停止控制
controller_interface::CallbackReturn GripperSuctionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// 更新
controller_interface::return_type GripperSuctionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // 1.从 实时缓冲区（RT_buffer） 读取 reference
  auto current_ref = input_ref_.readFromRT();

  // 2.控制硬件，写 command_interface
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    // 如果 reference 里有“有效的 displacement，就写到 command_interface
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      command_interfaces_[i].set_value((*current_ref)->displacements[i]);

      // 立刻把 displacement 清回 NaN
      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  // 3. 发布 controller 状态，把当前控制输出作为 state 通过 ~/state 发布出去
  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = abs(state_interfaces_[STATE_MY_ITFS].get_value()-command_interfaces_[CMD_MY_ITFS].get_value())>1e-6 ? false : true;
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

// 话题回调函数
void GripperSuctionController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // 如果消息里的 joint_names 数量和 控制器配置的关节数量一致
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg); // 把这条消息安全地存进 input_ref_（供实时控制循环使用）
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(),"收到 %zu 个接口，但controller预期包含 %zu 个接口。忽略此消息。",msg->joint_names.size(), params_.joints.size());
  }
}

}  // namespace gripper_suction_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gripper_suction_controller::GripperSuctionController, controller_interface::ControllerInterface)
