#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>

// 测试：
// 发送棋盘数据：ros2 topic pub --once /chess/points_json std_msgs/msg/String "{data: '{\"A1\": [0.353, 0.0, 0.242], \"A2\": [0.353, 0.200, 0.0242] }'}"

// 发送目标点：ros2 topic pub --once /ai_move_topic std_msgs/msg/String "{data: 'a1,a2,b1,b2'}"


class MachineryCommandControlNode : public rclcpp::Node
{
public:
    MachineryCommandControlNode() : Node("machinery_command_control_node")
    {
        // 参数
        this->declare_parameter("namespace","");
        namespace_ = this->get_parameter("namespace").as_string();
        this->declare_parameter("origin_position","[234.0,0.0,444.2]");
        std::string origin_position_str = this->get_parameter("origin_position").as_string();
        try
        {
            nlohmann::json origin_position_json = nlohmann::json::parse(origin_position_str);
            origin_position = origin_position_json.get<std::vector<double>>();
        }catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"origin_position的json格式解析错误：%s",e.what());
        }

        // 订阅棋盘点位
        chess_points_sub_ = this->create_subscription<std_msgs::msg::String>(
            "chess/points_json", 10,
            std::bind(&MachineryCommandControlNode::chess_points_json_callback, this, std::placeholders::_1));

        // 接收指令数据
        command_control_topic_ = this->create_subscription<std_msgs::msg::String>(
            "ai_move_topic",10,std::bind(&MachineryCommandControlNode::command_callback,this,std::placeholders::_1));

        // 发布机械臂目标点
        arm_command_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "cartesian_position_controller/command", 10);

        // 发布吸嘴控制指令
        gripper_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "gripper_controller/commands", 10);

        // 触发器
        machinery_pub_image_trigger = this->create_publisher<std_msgs::msg::Bool>(
            "machinery_pub_image_trigger",10);

        // QT debug信息
        qt_debug_msg_pub = this->create_publisher<std_msgs::msg::String>(
            "/status_topic",10);

        is_chess_points_received = false;
        RCLCPP_INFO(this->get_logger(), "机械臂指令控制节点已启动.");
    }

private:
    // --- 订阅回调 ---
    void chess_points_json_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!is_chess_points_received) {
            try
            {
                nlohmann::json chess_points_json = nlohmann::json::parse(msg->data);
                chess_points_.clear();
                for (auto &[key, value] : chess_points_json.items())
                {
                    chess_points_[key] = value.get<std::vector<double>>();
                }
                RCLCPP_INFO(this->get_logger(), "已成功接收并解析棋盘点位数据.");
                std_msgs::msg::String qt_msg = std_msgs::msg::String();
                qt_msg.data = '['+std::string(this->get_name())+']' + "已成功接收并解析棋盘点位数据";
                qt_debug_msg_pub->publish(qt_msg);
                // for (const auto& [key, coords] : chess_points_) {
                //     RCLCPP_INFO(this->get_logger(), "点 %s: [%.3f, %.3f, %.3f]",key.c_str(), coords[0], coords[1], coords[2]);
                // }
                is_chess_points_received = true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "JSON数据解析出错: %s", e.what());
                std_msgs::msg::String qt_msg = std_msgs::msg::String();
                qt_msg.data = '['+std::string(this->get_name())+']' + "JSON数据解析出错: "+e.what();
                qt_debug_msg_pub->publish(qt_msg);
            }
        }
    }

    void command_callback(std_msgs::msg::String msg) {
        RCLCPP_INFO(this->get_logger(), "收到新的移动序列任务");

        if (chess_points_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "棋盘坐标尚未接收，拒绝任务！");
            std_msgs::msg::String qt_msg = std_msgs::msg::String();
            qt_msg.data = '['+std::string(this->get_name())+']' + "棋盘坐标尚未接收，拒绝任务！";
            qt_debug_msg_pub->publish(qt_msg);
            return;
        }

        std::vector<std::string> point_to_move;
        size_t start = 0;
        size_t end = msg.data.find(',');
        // 循环处理逗号分隔的部分
        while (end != std::string::npos) {
            std::string token = msg.data.substr(start, end - start);

            size_t token_start = token.find_first_not_of(" \t");
            size_t token_end = token.find_last_not_of(" \t");

            if (token_start != std::string::npos) {
                point_to_move.push_back(token.substr(token_start, token_end - token_start + 1));
            }

            start = end + 1;
            end = msg.data.find(',', start);
        }

        // 处理最后一个点（因为后面没有逗号）
        std::string token = msg.data.substr(start);
        size_t token_start = token.find_first_not_of(" \t");
        size_t token_end = token.find_last_not_of(" \t");
        if (token_start != std::string::npos) {
            point_to_move.push_back(token.substr(token_start, token_end - token_start + 1));
        }
        RCLCPP_INFO(this->get_logger(), "收到新的移动序列任务，包含 %zu 个点", point_to_move.size());

        for (size_t i = 0; i < point_to_move.size(); ++i)
        {
            const auto &point_name = point_to_move[i];

            // 1. 检查点位是否存在
            std::vector<double> coords;
            {
                if (chess_points_.find(point_name) == chess_points_.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "点 '%s' 未找到，任务中止", point_name.c_str());
                    std_msgs::msg::String qt_msg = std_msgs::msg::String();
                    qt_msg.data = '['+std::string(this->get_name())+']' + "点 "+point_name+" 未找到，任务中止";
                    qt_debug_msg_pub->publish(qt_msg);

                    // 点位不存在时返回原点，并控制吸嘴
                    geometry_msgs::msg::PointStamped command_msg_0;
                    command_msg_0.header.stamp = this->get_clock()->now();
                    command_msg_0.header.frame_id = namespace_+"base_link";
                    command_msg_0.point.x = origin_position[0];
                    command_msg_0.point.y = origin_position[1];
                    command_msg_0.point.z = origin_position[2];
                    arm_command_pub_->publish(command_msg_0);
                    rclcpp::sleep_for(std::chrono::milliseconds(1500));
                    std_msgs::msg::Float64MultiArray suction_cmd;
                    suction_cmd.data={0.0};
                    gripper_command_pub_->publish(suction_cmd);

                    return;
                }
                coords = chess_points_[point_name];
            }

            // 检查点是否为特殊点（吃子）
            if (point_name=="o0"){
                // 去到中间点
                RCLCPP_INFO(this->get_logger(), "正在前往中间点");
                geometry_msgs::msg::PointStamped command_msg1;
                command_msg1.header.stamp = this->get_clock()->now();
                command_msg1.header.frame_id = namespace_+"base_link";
                command_msg1.point.x = (coords[0])*1000;
                command_msg1.point.y = (coords[1])*1000;
                command_msg1.point.z = (coords[2])*1000+150;
                arm_command_pub_->publish(command_msg1);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));

                // 去到投放点
                RCLCPP_INFO(this->get_logger(), "正在送到投放点");
                geometry_msgs::msg::PointStamped command_msg2;
                command_msg2.header.stamp = this->get_clock()->now();
                command_msg2.header.frame_id = namespace_+"base_link";
                command_msg2.point.x = (coords[0])*1000;
                command_msg2.point.y = (coords[1])*1000;
                command_msg2.point.z = (coords[2])*1000;
                arm_command_pub_->publish(command_msg2);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));

                // 控制吸嘴
                RCLCPP_INFO(this->get_logger(), "设置吸嘴状态为: 0");
                std_msgs::msg::Float64MultiArray suction_cmd;
                suction_cmd.data={0.0};
                gripper_command_pub_->publish(suction_cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(1000));

                // 返回原点
                RCLCPP_INFO(this->get_logger(), "正在前往中间点");
                geometry_msgs::msg::PointStamped command_msg3;
                command_msg3.header.stamp = this->get_clock()->now();
                command_msg3.header.frame_id = namespace_+"base_link";
                command_msg3.point.x = (coords[0])*1000;
                command_msg3.point.y = (coords[1])*1000;
                command_msg3.point.z = (coords[2])*1000+150;
                arm_command_pub_->publish(command_msg3);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));
            }
            else{
                // 2. 发送移动指令
                geometry_msgs::msg::PointStamped command_msg1;
                command_msg1.header.stamp = this->get_clock()->now();
                command_msg1.header.frame_id = namespace_+"base_link";

                command_msg1.point.x = (coords[0])*1000;
                command_msg1.point.y = (coords[1])*1000;
                command_msg1.point.z = (coords[2])*1000+30;
                arm_command_pub_->publish(command_msg1);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));

                geometry_msgs::msg::PointStamped command_msg2;
                command_msg2.header.stamp = this->get_clock()->now();
                command_msg2.header.frame_id = namespace_+"base_link";

                command_msg2.point.x = (coords[0])*1000;
                command_msg2.point.y = (coords[1])*1000;
                command_msg2.point.z = (coords[2])*1000;

                RCLCPP_INFO(this->get_logger(), "正在前往点 '%s': (%.1f, %.1f, %.1f)",
                            point_name.c_str(), command_msg2.point.x, command_msg2.point.y, command_msg2.point.z);
                arm_command_pub_->publish(command_msg2);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));

                // 3. 到达目标点后，控制吸嘴
                if (i%2==0) {
                    RCLCPP_INFO(this->get_logger(), "设置吸嘴状态为: 1");
                    std_msgs::msg::Float64MultiArray suction_cmd;
                    suction_cmd.data={1.0};
                    gripper_command_pub_->publish(suction_cmd);
                }
                else if (i%2==1){
                    RCLCPP_INFO(this->get_logger(), "设置吸嘴状态为: 0");
                    std_msgs::msg::Float64MultiArray suction_cmd;
                    suction_cmd.data={0.0};
                    gripper_command_pub_->publish(suction_cmd);
                }
                rclcpp::sleep_for(std::chrono::milliseconds(1000));

                // 4.返回中间点
                geometry_msgs::msg::PointStamped command_msg_3;
                command_msg_3.header.stamp = this->get_clock()->now();
                command_msg_3.header.frame_id = namespace_+"base_link";
                command_msg_3.point.x = (coords[0])*1000;
                command_msg_3.point.y = (coords[1])*1000;
                command_msg_3.point.z = (coords[2])*1000+30;
                arm_command_pub_->publish(command_msg_3);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));
            }
        }

        // 5.返回原点
        geometry_msgs::msg::PointStamped command_msg_4;
        command_msg_4.header.stamp = this->get_clock()->now();
        command_msg_4.header.frame_id = namespace_+"base_link";
        command_msg_4.point.x = origin_position[0];
        command_msg_4.point.y = origin_position[1];
        command_msg_4.point.z = origin_position[2];
        arm_command_pub_->publish(command_msg_4);
        RCLCPP_INFO(this->get_logger(), "下棋完毕，正在前往原点");
        std_msgs::msg::String qt_msg = std_msgs::msg::String();
        qt_msg.data = '['+std::string(this->get_name())+']' + "下棋完毕，正在前往原点";
        qt_debug_msg_pub->publish(qt_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(1500));

        // 控制吸嘴，防止出意外
        std_msgs::msg::Float64MultiArray suction_cmd;
        suction_cmd.data={0.0};
        gripper_command_pub_->publish(suction_cmd);

        // 6.触发相机拍摄照片
        std_msgs::msg::Bool triggerSignal;
        triggerSignal.data=true;
        machinery_pub_image_trigger->publish(triggerSignal);
    }

    // ROS 接口
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chess_points_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_control_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr arm_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr machinery_pub_image_trigger;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qt_debug_msg_pub;

    // 参数
    std::map<std::string, std::vector<double>> chess_points_;
    bool is_chess_points_received;
    std::string namespace_;
    std::vector<double> origin_position;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 使用多线程执行器，确保Action服务器和其他回调不会相互阻塞
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<MachineryCommandControlNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
