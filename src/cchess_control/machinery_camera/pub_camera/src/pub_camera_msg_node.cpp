#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

class PubCameraMsgNode : public rclcpp::Node
{
public:
  PubCameraMsgNode() :Node("pub_camera_msg_node"){
    // 参数
    this->declare_parameter("namespace", "");
    namespace_ = this->get_parameter("namespace").as_string();

    // 话题
    color_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",10,
      std::bind(&PubCameraMsgNode::color_image_callback,this,std::placeholders::_1));
    person_pub_image_trigger = this->create_subscription<std_msgs::msg::Bool>("person_pub_image_trigger",10,
      std::bind(&PubCameraMsgNode::person_color_image_callback,this,std::placeholders::_1));
    machinery_pub_image_trigger = this->create_subscription<std_msgs::msg::Bool>("machinery_pub_image_trigger",10,
      std::bind(&PubCameraMsgNode::machinery_color_image_callback,this,std::placeholders::_1));
    after_move_color_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image_after_move",10);
    before_move_color_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image_before_move",10);
  }

private:
  void color_image_callback(sensor_msgs::msg::Image image) {
    color_image = image;
  }

  void machinery_color_image_callback(std_msgs::msg::Bool trigger) {
    RCLCPP_INFO(this->get_logger(),"接收到机器信号：%d",trigger.data);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (trigger.data) {
      before_move_color_image_publisher->publish(color_image);
    }
  }

  void person_color_image_callback(std_msgs::msg::Bool trigger) {
    RCLCPP_INFO(this->get_logger(),"接收到人信号：%d",trigger.data);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (trigger.data) {
      after_move_color_image_publisher->publish(color_image);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_subscriber;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr machinery_pub_image_trigger;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr person_pub_image_trigger;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr after_move_color_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr before_move_color_image_publisher;
  sensor_msgs::msg::Image color_image;
  std::string namespace_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PubCameraMsgNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
