#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <realtime_tools/realtime_box.hpp>
#include <atomic>
#include <gripper_suction_controller/gripper_suction_controller.hpp>
#include <machinery_control_msg/msg/task_status.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// 测试：
// 发送棋盘数据：ros2 topic pub --once /chess/points_json std_msgs/msg/String "{data: '{\"A1\": [0.353, 0.0, 0.242], \"A2\": [0.353, 0.200, 0.0242] }'}"

// 发送目标点：ros2 topic pub --once /left/ai_move_topic std_msgs/msg/String "{data: 'a1,a2,b1,b2'}"

// 状态机，用于表示机械臂状态
enum class MachineryStatus
{
    IDLE,
    BUSY
};

class MachineryCommandControlNode : public rclcpp::Node
{
public:
    MachineryCommandControlNode() : Node("machinery_command_control_node")
    {
        // 参数
        this->declare_parameter("namespace","");
        namespace_ = this->get_parameter("namespace").as_string();
        this->declare_parameter("origin_position","[180.0,0.0,444.2]");
        std::string origin_position_str = this->get_parameter("origin_position").as_string();
        try
        {
            nlohmann::json origin_position_json = nlohmann::json::parse(origin_position_str);
            origin_position = origin_position_json.get<std::vector<double>>();
        }catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"origin_position的json格式解析错误：%s",e.what());
        }
        this->declare_parameter("custom_origin_position","[180.0,0.0,444.2]");
        std::string custom_origin_position_str = this->get_parameter("custom_origin_position").as_string();
        try
        {
            nlohmann::json custom_origin_position_json = nlohmann::json::parse(custom_origin_position_str);
            custom_origin_position = custom_origin_position_json.get<std::vector<double>>();
        }catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),"custom_origin_position的json格式解析错误：%s",e.what());
        }

        // 变量
        error = 1e-6;
        last_cartesian_command = {-1, -1, -1};
        last_gripper_command = -1;
        is_chess_points_received = false;
        last_task_status[0] = true;
        last_task_status[1] = true;
        is_running = false;
        machinery_status_.set(MachineryStatus::IDLE);

        // 订阅棋盘点位
        chess_points_sub_ = this->create_subscription<std_msgs::msg::String>(
            "chess/points_json", 10,
            std::bind(&MachineryCommandControlNode::chess_points_json_callback, this, std::placeholders::_1));

        // 接收指令数据
        command_control_topic_ = this->create_subscription<std_msgs::msg::String>(
            "ai_move_topic",10,std::bind(&MachineryCommandControlNode::process_task,this,std::placeholders::_1));

        // 发布机械臂目标点
        arm_command_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "cartesian_position_controller/command", 10);

        // 发布吸嘴控制指令
        // gripper_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        //     "gripper_controller/commands", 10);
        gripper_command_pub_ = this->create_publisher<gripper_suction_controller::GripperSuctionController::ControllerReferenceMsg>(
            "gripper_controller/reference", 10);

        // 触发器
        machinery_pub_image_trigger = this->create_publisher<std_msgs::msg::Bool>(
            "machinery_pub_image_trigger",10);

        // QT debug信息
        qt_debug_msg_pub = this->create_publisher<std_msgs::msg::String>(
            "/status_topic",10);

        // 接收机械臂状态
        arm_status_sub_.subscribe(this,"cartesian_position_controller/task_status");
        // 接收吸嘴状态
        gripper_status_sub_.subscribe(this,"gripper_controller/state");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10),arm_status_sub_,gripper_status_sub_);
        sync_->registerCallback(std::bind(&MachineryCommandControlNode::task_status_callback,this,std::placeholders::_1,std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "机械臂指令控制节点已启动.");
    }

    ~MachineryCommandControlNode()
    {
    }
private:
    // --- 工具函数 ---
    void control_cartesian_point(geometry_msgs::msg::PointStamped point_stamped)
    {
        // 等待机械臂空闲才进入函数
        MachineryStatus status;
        machinery_status_.get(status);
        while (status != MachineryStatus::IDLE)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            machinery_status_.get(status);
            RCLCPP_INFO(this->get_logger(),"机械臂繁忙！");
        }

        // 只有发送命令不一致的时候才需要 发布指令 和 修改机械臂状态
        if (abs(last_cartesian_command[0] - point_stamped.point.x)>error ||
            abs(last_cartesian_command[1] - point_stamped.point.y)>error ||
            abs(last_cartesian_command[2] - point_stamped.point.z)>error)
        {
            // 发布指令
            arm_command_pub_->publish(point_stamped);

            // 修改机械臂状态
            machinery_status_.set(MachineryStatus::BUSY);
            last_cartesian_command[0] = point_stamped.point.x;
            last_cartesian_command[1] = point_stamped.point.y;
            last_cartesian_command[2] = point_stamped.point.z;
        }

        // 等待机械臂空闲才退出函数
        machinery_status_.get(status);
        while (status != MachineryStatus::IDLE)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            machinery_status_.get(status);
        }
    }

    void control_gripper(gripper_suction_controller::GripperSuctionController::ControllerReferenceMsg suction_cmd)
    {
        // 等待机械臂空闲才进入函数
        MachineryStatus status;
        machinery_status_.get(status);
        while (status != MachineryStatus::IDLE)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            machinery_status_.get(status);
        }

        // 只有发送命令不一致的时候才需要 发布指令 和 修改机械臂状态
        if (abs(last_gripper_command - suction_cmd.displacements[0])>error)
        {
            // 发布指令
            gripper_command_pub_->publish(suction_cmd);

            // 修改机械臂状态
            machinery_status_.set(MachineryStatus::BUSY);
            last_gripper_command = suction_cmd.displacements[0];
        }

        // 等待机械臂空闲才退出函数
        machinery_status_.get(status);
        while (status != MachineryStatus::IDLE)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            machinery_status_.get(status);
        }
    }

    // --- 订阅回调 ---
    void task_status_callback(const machinery_control_msg::msg::TaskStatus::ConstSharedPtr& arm_status, const gripper_suction_controller::GripperSuctionController::ControllerStateMsg::ConstSharedPtr& gripper_status)
    {
        if ((last_task_status[0]==true && arm_status->is_idle==false) ||
            (last_task_status[1]==true && gripper_status->set_point==0.0))
        {
            machinery_status_.set(MachineryStatus::BUSY);
            RCLCPP_INFO(this->get_logger(),"机械臂状态：BUSY");
        }
        else if ((last_task_status[0]==false && arm_status->is_idle==true) ||
                 (last_task_status[1]==false && gripper_status->set_point==1.0))
        {
            machinery_status_.set(MachineryStatus::IDLE);
            RCLCPP_INFO(this->get_logger(),"机械臂状态：IDLE");
        }
        last_task_status[0] = arm_status->is_idle;
        last_task_status[1] = (gripper_status->set_point > 0.5) ? true : false;
    }

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

    void process_task(std_msgs::msg::String msg)
    {
        if (is_running)
        {
            RCLCPP_WARN(this->get_logger(),"任务正在执行中，忽略此任务！");
            return;
        }

        is_running = true;
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

        std::thread(&MachineryCommandControlNode::command_callback, this,point_to_move).detach();
    }
    void command_callback(std::vector<std::string> point_to_move) {
        struct StateGuard {
            std::atomic<bool>& flag;
            StateGuard(std::atomic<bool>& f) : flag(f) { flag = true; }
            ~StateGuard() { flag = false; }
        } state_guard(is_running);

        geometry_msgs::msg::PointStamped command_msg;
        command_msg.header.frame_id = namespace_+"base_link";
        gripper_suction_controller::GripperSuctionController::ControllerReferenceMsg suction_cmd;
        suction_cmd.header.frame_id = namespace_+"base_link";

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

                    // 点位不存在时返回自定义原点，并控制吸嘴
                    command_msg.header.stamp = this->get_clock()->now();
                    command_msg.point.x = custom_origin_position[0];
                    command_msg.point.y = custom_origin_position[1];
                    command_msg.point.z = custom_origin_position[2];
                    control_cartesian_point(command_msg);

                    suction_cmd.header.stamp = this->get_clock()->now();
                    suction_cmd.joint_names={"gripper_suction"};
                    suction_cmd.displacements={0.0};
                    control_gripper(suction_cmd);

                    return;
                }
                coords = chess_points_[point_name];
            }

            // 检查点是否为特殊点（吃子）
            if (point_name=="o0"){
                // 去到中间点
                RCLCPP_INFO(this->get_logger(), "正在前往中间点");
                command_msg.header.stamp = this->get_clock()->now();
                command_msg.point.x = (coords[0])*1000;
                command_msg.point.y = (coords[1])*1000;
                command_msg.point.z = (coords[2])*1000+90;
                control_cartesian_point(command_msg);

                // 去到投放点
                RCLCPP_INFO(this->get_logger(), "正在送到投放点");
                command_msg.header.stamp = this->get_clock()->now();
                command_msg.point.x = (coords[0])*1000;
                command_msg.point.y = (coords[1])*1000;
                command_msg.point.z = (coords[2])*1000;
                control_cartesian_point(command_msg);

                // 控制吸嘴
                RCLCPP_INFO(this->get_logger(), "设置吸嘴状态为: 0");
                suction_cmd.header.stamp = this->get_clock()->now();
                suction_cmd.joint_names={"gripper_suction"};
                suction_cmd.displacements={0.0};
                control_gripper(suction_cmd);

                // 返回中间点
                RCLCPP_INFO(this->get_logger(), "正在前往中间点");
                command_msg.header.stamp = this->get_clock()->now();
                command_msg.header.frame_id = namespace_+"base_link";
                command_msg.point.x = (coords[0])*1000;
                command_msg.point.y = (coords[1])*1000;
                command_msg.point.z = (coords[2])*1000+20;
                control_cartesian_point(command_msg);
            }
            else{
                // 2. 发送移动指令
                // 去到中间点
                RCLCPP_INFO(this->get_logger(), "去中间点");
                command_msg.header.stamp = this->get_clock()->now();
                command_msg.point.x = (coords[0])*1000;
                command_msg.point.y = (coords[1])*1000;
                command_msg.point.z = (coords[2])*1000+20;
                control_cartesian_point(command_msg);

                // 去到投放点
                command_msg.header.stamp = this->get_clock()->now();
                command_msg.point.x = (coords[0])*1000;
                command_msg.point.y = (coords[1])*1000;
                command_msg.point.z = (coords[2])*1000;
                control_cartesian_point(command_msg);

                // 3. 到达目标点后，控制吸嘴
                if (i%2==0) {
                    RCLCPP_INFO(this->get_logger(), "设置吸嘴状态为: 1");
                    suction_cmd.header.stamp = this->get_clock()->now();
                    suction_cmd.joint_names={"gripper_suction"};
                    suction_cmd.displacements={1.0};
                    control_gripper(suction_cmd);
                }
                else if (i%2==1){
                    RCLCPP_INFO(this->get_logger(), "设置吸嘴状态为: 0");
                    suction_cmd.header.stamp = this->get_clock()->now();
                    suction_cmd.joint_names={"gripper_suction"};
                    suction_cmd.displacements={0.0};
                    control_gripper(suction_cmd);
                }

                // 4.返回中间点
                RCLCPP_INFO(this->get_logger(), "返回中间点");
                command_msg.header.stamp = this->get_clock()->now();
                command_msg.point.x = (coords[0])*1000;
                command_msg.point.y = (coords[1])*1000;
                command_msg.point.z = (coords[2])*1000+20;
                control_cartesian_point(command_msg);
            }
        }

        // 5.返回原点
        RCLCPP_INFO(this->get_logger(), "返回原点");
        command_msg.header.stamp = this->get_clock()->now();
        command_msg.point.x = origin_position[0];
        command_msg.point.y = origin_position[1];
        command_msg.point.z = origin_position[2];
        control_cartesian_point(command_msg);

        // 6.返回自定义原点
        command_msg.header.stamp = this->get_clock()->now();
        command_msg.point.x = custom_origin_position[0];
        command_msg.point.y = custom_origin_position[1];
        command_msg.point.z = custom_origin_position[2];
        control_cartesian_point(command_msg);
        RCLCPP_INFO(this->get_logger(), "下棋完毕，正在前往原点");
        std_msgs::msg::String qt_msg = std_msgs::msg::String();
        qt_msg.data = '['+std::string(this->get_name())+']' + "下棋完毕，正在前往原点";
        qt_debug_msg_pub->publish(qt_msg);

        // 7.控制吸嘴，防止出意外
        suction_cmd.header.stamp = this->get_clock()->now();
        suction_cmd.joint_names={"gripper_suction"};
        suction_cmd.displacements={0.0};
        control_gripper(suction_cmd);

        // 8.触发相机拍摄照片
        std_msgs::msg::Bool triggerSignal;
        triggerSignal.data=true;
        machinery_pub_image_trigger->publish(triggerSignal);
    }

    // ROS 接口
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chess_points_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_control_topic_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr arm_command_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<gripper_suction_controller::GripperSuctionController::ControllerReferenceMsg>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr machinery_pub_image_trigger;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qt_debug_msg_pub;

    message_filters::Subscriber<machinery_control_msg::msg::TaskStatus> arm_status_sub_;
    message_filters::Subscriber<gripper_suction_controller::GripperSuctionController::ControllerStateMsg> gripper_status_sub_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<machinery_control_msg::msg::TaskStatus,gripper_suction_controller::GripperSuctionController::ControllerStateMsg>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 变量
    realtime_tools::RealtimeBox<MachineryStatus> machinery_status_{MachineryStatus::IDLE}; // 无锁、实时安全的数据交换盒
    double error;
    std::vector<double> last_cartesian_command;
    double last_gripper_command;
    bool last_task_status[2];
    bool is_chess_points_received;
    std::atomic<bool> is_running;

    // 参数
    std::map<std::string, std::vector<double>> chess_points_;
    std::string namespace_;
    std::vector<double> origin_position;
    std::vector<double> custom_origin_position;
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
