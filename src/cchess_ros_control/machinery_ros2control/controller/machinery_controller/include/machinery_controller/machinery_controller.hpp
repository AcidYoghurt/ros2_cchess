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

#ifndef MACHINERY_CONTROLLER__MACHINERY_CONTROLLER_HPP_
#define MACHINERY_CONTROLLER__MACHINERY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "machinery_control_msg/msg/task_status.hpp"

namespace machinery_controller
{
using CmdType = geometry_msgs::msg::PointStamped;

class MachineryController : public controller_interface::ControllerInterface
{
public:
  MachineryController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_ = nullptr;
  rclcpp::Publisher<machinery_control_msg::msg::TaskStatus>::SharedPtr task_status_publisher_ = nullptr;

  // 变量
  realtime_tools::RealtimeBox<std::shared_ptr<CmdType>> received_command_ptr_{nullptr}; // 无锁、实时安全的数据交换盒，这里用于存储业务代码调用controller的数据，即要发送到串口的目标点
  std::vector<double> last_command_interfaces_;
  machinery_control_msg::msg::TaskStatus task_status_;
  double epsilon;

  // 参数
  std::string cartesian_joint_name_;
};

}  // namespace machinery_controller

#endif  // MACHINERY_CONTROLLER__MACHINERY_CONTROLLER_HPP_
