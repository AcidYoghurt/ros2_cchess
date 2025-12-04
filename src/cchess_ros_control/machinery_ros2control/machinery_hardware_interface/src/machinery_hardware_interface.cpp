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

#include <limits>
#include <vector>
#include <memory>
#include <functional>
#include <libserial/SerialPort.h>
#include <cstdio>
#include <regex>
#include <cmath>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "machinery_hardware_interface/machinery_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace machinery_hardware_interface
{
    /**
     * @说明 默认构造函数，没什么用，只是用来初始化logger
     */
    MachineryHardwareInterface::MachineryHardwareInterface() : logger_(rclcpp::get_logger("machinery_hardware_interface_node"))
    {
    }

    /**
     * @说明 重启串口函数
     */
    bool MachineryHardwareInterface::restart_serial() {
        RCLCPP_WARN(logger_, "检测到串口错误，尝试重启串口...");
        const auto retry_delay = std::chrono::milliseconds(500);
 
        while (true)
        {
            // 放弃对旧串口对象的所有权，以避免在错误状态下调用其析构函数导致崩溃。
            // 这会造成一次小的内存泄漏，但保证了节点的健壮性。
            if (serial_port_) {
                serial_port_.release();
            }
            
            // 1. 创建一个新的串口对象
            serial_port_ = std::make_unique<LibSerial::SerialPort>();
 
            // 2. 尝试打开并配置新的串口对象
            try
            {
                serial_port_->Open(serial_port_name_);
                serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
                serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
                serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
                serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
 
                RCLCPP_INFO(logger_, "串口重启成功，端口：%s。", serial_port_name_.c_str());
                return true; // 成功，立即返回
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(logger_, "重连尝试失败: %s", e.what());
                rclcpp::sleep_for(retry_delay);
            }
        }
        RCLCPP_FATAL(logger_, "串口重连彻底失败。");
        return false;
    }

    /**
     * @说明 初始化,获取 URDF 中的参数
     * @param info
     * @return
     */
    hardware_interface::CallbackReturn MachineryHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // 从URDF中获取硬件参数
        frame_prefix_ = info_.hardware_parameters["frame_prefix"];
        serial_port_name_ = info_.hardware_parameters["serial_port_name"];
        // baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        std::string origin_position_str = info_.hardware_parameters["origin_position"];
        try
        {
            nlohmann::json origin_position_json = nlohmann::json::parse(origin_position_str);
            origin_position = origin_position_json.get<std::vector<double>>();
        }catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_,"origin_position的json格式解析错误：%s",e.what());
        }

        // 初始化硬件状态/命令变量
        hw_state_x_ = origin_position[0];
        hw_state_y_ = origin_position[1];
        hw_state_z_ = origin_position[2];
        hw_command_x_ = hw_state_x_;
        hw_command_y_ = hw_state_y_;
        hw_command_z_ = hw_state_z_;
        last_hw_command_x_ = 0.0;
        last_hw_command_y_ = 0.0;
        last_hw_command_z_ = 0.0;
        hw_state_suction_ = 0.0;
        hw_command_suction_ = hw_state_suction_;
        last_hw_command_suction_ = 1.0;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(std::make_shared<rclcpp::Node>("machinery_hardware_interface_node"));

        const hardware_interface::ComponentInfo &joint = info_.joints[0];
        // 检查URDF中定义的接口数量是否正确
        if (joint.command_interfaces.size() != 3)
        {
            RCLCPP_FATAL(
                logger_, "Joint '%s' has %zu command interfaces found. 3 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces found. 3 expected.", joint.name.c_str(),joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    /**
     * @说明 控制硬件的代码，初始化比如说PWN，I2C等接口（该函数会在 on_init 后自动调用）
     * @return
     */
    hardware_interface::CallbackReturn MachineryHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // 初始化串口对象和连接
        serial_port_ = std::make_unique<LibSerial::SerialPort>();
        try
        {
            serial_port_->Open(serial_port_name_);
            serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "无法打开串口: %s", e.what());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(logger_, "初始化完成");
        return CallbackReturn::SUCCESS;
    }

    /**
     * @说明 导出state_interface（例如，当前关节位置）
     * @brief 告诉 ros2_control 我们可以提供哪些状态信息
     * @return
     */
    std::vector<hardware_interface::StateInterface> MachineryHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Correctly export the three Cartesian state interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, "x_position", &hw_state_x_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, "y_position", &hw_state_y_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, "z_position", &hw_state_z_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[1].name, "enable", &hw_state_suction_));

        return state_interfaces;
    }

    /**
     * @说明 导出command_interface（例如，目标关节位置）
     * @brief 告诉 ros2_control 我们可以接收哪些指令
     * @return
     */
    std::vector<hardware_interface::CommandInterface> MachineryHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Correctly export the three Cartesian command interfaces
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, "x_position", &hw_command_x_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, "y_position", &hw_command_y_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, "z_position", &hw_command_z_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[1].name, "enable", &hw_command_suction_));

        return command_interfaces;
    }

    /**
     * @说明 在控制器启动时调用。
     * @brief on_activate之后，我们的 hardwareinterface 就要开始工作了，所以这里要统一 state_interface 和 command_interface的值
     * @return
     */
    hardware_interface::CallbackReturn MachineryHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // 在激活时，将硬件状态设置原点
        hw_state_x_ = origin_position[0];
        hw_state_y_ = origin_position[1];
        hw_state_z_ = origin_position[2];
        hw_command_x_ = hw_state_x_;
        hw_command_y_ = hw_state_y_;
        hw_command_z_ = hw_state_z_;
        last_hw_command_x_ = 0.0;
        last_hw_command_y_ = 0.0;
        last_hw_command_z_ = 0.0;
        hw_state_suction_ = 0.0;
        hw_command_suction_ = hw_state_suction_;
        last_hw_command_suction_ = 1.0;
        RCLCPP_INFO(logger_, "on_activate函数执行");
        return CallbackReturn::SUCCESS;
    }

    /**
     * @说明 在控制器停止时调用
     * @return
     */
    hardware_interface::CallbackReturn MachineryHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // 急停
        std::string suction_command = "Suction_" + std::to_string(0) + ",";
        serial_port_->Write(suction_command);
        serial_port_->DrainWriteBuffer();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        // std::string arm_command = "DescartesPoint_" + std::to_string(origin_position[0]) + "," +
        //                             std::to_string(origin_position[1]) + "," +
        //                             std::to_string(origin_position[2]) + "," + "100";
        // serial_port_->Write(arm_command);
        // serial_port_->DrainWriteBuffer();

        // 关闭串口
        if (serial_port_ && serial_port_->IsOpen())
        {
            serial_port_->Close();
        }
        return CallbackReturn::SUCCESS;
    }

    /**
     * @说明 从硬件控制器读取数据，存放到 state_interface
     * @return
     */
    hardware_interface::return_type MachineryHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        try
        {
            if (!serial_port_ || !serial_port_->IsOpen())
            {
                // 如果串口未初始化或未打开，则不执行读取操作
                return hardware_interface::return_type::OK;
            }
            // 清空 response 字符串以接收新数据
            response.clear();
            serial_port_->ReadLine(response, '\n', 10);
            RCLCPP_INFO(logger_, "串口返回数据：%s",response.c_str());

            double x = 0, y = 0, z = 0, suction = 0.0;
            if (!response.empty() && sscanf(response.c_str(),"DescartesPointOffset: %lfmm(X), %lfmm(Y), %lfmm(Z)\r\n",&x, &y, &z) == 3) {
              hw_state_x_ = x;
              hw_state_y_ = y;
              hw_state_z_ = z;

              last_hw_command_x_ = hw_state_x_;
              last_hw_command_y_ = hw_state_y_;
              last_hw_command_z_ = hw_state_z_;
              // RCLCPP_INFO(logger_, "返回笛卡尔积坐标：%f, %f, %f", hw_state_x_, hw_state_y_, hw_state_z_);
            } else if (!response.empty() && sscanf(response.c_str(), "Suction: %lf\r\n", &suction) == 1) {
              hw_state_suction_ = suction;

              // 更新上一次发送的值
              last_hw_command_suction_ = hw_command_suction_;
              // RCLCPP_INFO(logger_, "返回吸嘴状态：%f", hw_state_suction_);
            }
        }
        catch (const std::exception &e)
        {
            std::string error_msg(e.what());
            if (error_msg.find("Input/output error") != std::string::npos || error_msg.find("Inappropriate ioctl for device") != std::string::npos) {
                RCLCPP_ERROR(logger_, "读取串口时发生I/O错误: %s", e.what());
                restart_serial();
            } else if (error_msg.find("Read timeout") == std::string::npos) {
                RCLCPP_ERROR(logger_, "读取串口出错: %s", e.what());
            }
        }

        // 发布TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = rclcpp::Clock().now();
        t.header.frame_id = frame_prefix_+"base_link";
        t.child_frame_id = frame_prefix_+"link5";
        t.transform.translation.x = hw_state_x_ / 1000;
        t.transform.translation.y = hw_state_y_ / 1000;
        t.transform.translation.z = hw_state_z_ / 1000;

        // 绕X旋转90度
        tf2::Quaternion base_rotation;
        base_rotation.setX(0.707);
        base_rotation.setY(0.0);
        base_rotation.setZ(0.0);
        base_rotation.setW(0.707);

        // 绕Y旋转...度
        double radians = std::atan2(t.transform.translation.y, t.transform.translation.x);
        tf2::Quaternion y_rotation;
        y_rotation.setRPY(0, radians, 0);

        // 四元数相乘
        tf2::Quaternion final_rotation = base_rotation * y_rotation;

        // 最终的rotation
        t.transform.rotation.x = final_rotation.x();
        t.transform.rotation.y = final_rotation.y();
        t.transform.rotation.z = final_rotation.z();
        t.transform.rotation.w = final_rotation.w();

        tf_broadcaster_->sendTransform(t);
        return hardware_interface::return_type::OK;
    }

    /**
     * @说明 获取 command_interface 的值，传递给硬件控制器
     * @return
     */
    hardware_interface::return_type MachineryHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // TODO(anyone): write robot's commands'
        try
        {
            if (!serial_port_ || !serial_port_->IsOpen())
            {
                RCLCPP_ERROR(logger_, "串口未打开！");
                return hardware_interface::return_type::ERROR;
            }

            const double epsilon = 1e-6;

            // 发送控制模式指令
            if (std::abs(hw_command_x_ - last_hw_command_x_) > epsilon ||
                std::abs(hw_command_y_ - last_hw_command_y_) > epsilon ||
                std::abs(hw_command_z_ - last_hw_command_z_) > epsilon)
            {
                RCLCPP_INFO(logger_, "指令变化，写入笛卡尔坐标：%f, %f, %f", hw_command_x_, hw_command_y_, hw_command_z_);
                std::string arm_command = "DescartesPoint_" + std::to_string(hw_command_x_) + "," +
                                          std::to_string(hw_command_y_) + "," +
                                          std::to_string(hw_command_z_) + "," + "100";
                serial_port_->Write(arm_command);
                serial_port_->DrainWriteBuffer();
            }

            // 发送吸嘴指令
            if (std::abs(hw_command_suction_ - last_hw_command_suction_) > epsilon)
            {
                RCLCPP_INFO(logger_, "指令变化，写入吸嘴状态：%f", hw_command_suction_);
                std::string suction_command = "Suction_" + std::to_string(int(hw_command_suction_)) + ",";
                serial_port_->Write(suction_command);
                serial_port_->DrainWriteBuffer();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "写入串口出错: %s", e.what());
            std::string error_msg(e.what());
            if (error_msg.find("Input/output error") != std::string::npos) {
                restart_serial();
            }
        }
        return hardware_interface::return_type::OK;
    }
} // namespace machinery_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    machinery_hardware_interface::MachineryHardwareInterface, hardware_interface::SystemInterface);
