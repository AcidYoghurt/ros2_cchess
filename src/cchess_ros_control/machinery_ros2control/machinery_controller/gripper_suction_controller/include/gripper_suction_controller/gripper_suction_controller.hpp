// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef GRIPPER_SUCTION_CONTROLLER__GRIPPER_SUCTION_CONTROLLER_HPP_
#define GRIPPER_SUCTION_CONTROLLER__GRIPPER_SUCTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "gripper_suction_controller_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace gripper_suction_controller
{
// 状态接口名称常量（用于定义要读取硬件的哪些状态）
static constexpr size_t STATE_MY_ITFS = 0;

// 命令接口名称常量（用于定义要控制硬件的哪些命令）
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class GripperSuctionController : public controller_interface::ControllerInterface
{
public:
  GripperSuctionController();

   // 1. 初始化：在控制器加载时调用。通常用于声明参数、初始化成员变量。
  controller_interface::CallbackReturn on_init() override;

  // 定义控制器需要使用的“命令接口”（Command Interfaces），例如：向硬件写入位置、速度或吸盘开关信号。
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // 定义控制器需要使用的“状态接口”（State Interfaces），例如：从硬件读取当前位置或传感器状态。
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // 2. 配置：在控制器进入 'inactive' 状态前调用。
  // 通常用于读取参数、设置订阅者（Subscribers）和发布者（Publishers）、分配内存等。
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // 3. 激活：在控制器开始运行（进入 'active' 状态）时调用。
  // 通常用于重置控制器状态、清除缓冲区，准备开始实时控制循环。
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 4. 停用：在控制器停止运行（退出 'active' 状态）时调用。
  // 通常用于发送停止指令（如关闭吸盘），确保硬件处于安全状态。
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 5. 更新：这是控制器的核心循环函数，以高频（实时）运行。
  // 在这里读取状态接口（state_interfaces_），计算控制逻辑，并写入命令接口（command_interfaces_）。
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  // 定义控制器使用的消息类型别名
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  // 参数监听器：用于处理 ROS 2 参数的动态更新
  std::shared_ptr<gripper_suction_controller::ParamListener> param_listener_;
  gripper_suction_controller::Params params_;

  std::vector<std::string> state_joints_;

  // 订阅者：接收外部发来的控制指令（非实时线程）
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  // 实时缓冲区：用于在“非实时订阅回调”和“实时 update 循环”之间安全地传递数据，避免锁竞争。
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  // 发布者：发布控制器的内部状态（实时线程安全发布）
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // 话题回调函数：当收到控制指令时触发
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace gripper_suction_controller

#endif  // GRIPPER_SUCTION_CONTROLLER__GRIPPER_SUCTION_CONTROLLER_HPP_
