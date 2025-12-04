import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool,String
from cv_bridge import CvBridge
import cv2
import time
import threading

class ChessCameraPublisher(Node):
    def __init__(self):
        super().__init__('chess_camera_publisher')
        
        # 创建两个发布者，分别用于移动前和移动后的图像
        self.publisher_before = self.create_publisher(Image, 'image_before_move', 10)
        self.publisher_after = self.create_publisher(Image, 'image_after_move', 10)
        self.publisher_statues = self.create_publisher(Image, 'status_topic', 10)
        self.bridge = CvBridge()
        self.get_logger().info('中国象棋摄像头发布器已启动')
        
        # 存储捕获的图像
        self.before_image = None
        self.after_image = None
        
        # 摄像头状态
        self.camera = None
        self.camera_active = False
        self.current_frame = None
        
        # 启动摄像头
        if not self.start_camera():
            self.get_logger().error("无法启动摄像头！")
            return
        
        # 创建摄像头线程
        self.camera_thread = threading.Thread(target=self.camera_capture_thread)
        self.camera_thread.daemon = True
        self.camera_thread.start()

        
        # 创建订阅者用于接收触发信号
        self.subscription_machinery = self.create_subscription(
            Bool,
            'machinery_pub_image_trigger',
            self.machinery_trigger_callback,
            10
        )
        
        self.subscription_person = self.create_subscription(
            Bool,
            'person_pub_image_trigger',
            self.person_trigger_callback,
            10
        )

    def start_camera(self):
        """启动摄像头"""
        if self.camera is None:
            try:
                #这里更改相机设备（video0就是0,video1就是1,深度相机会有两个相机设备，一般第一个是彩色相机）
                self.camera = cv2.VideoCapture(0)
                if not self.camera.isOpened():
                    self.get_logger().error("无法打开摄像头！")
                    status_msg=String()
                    status_msg.data="无法打开摄像头！"
                    self.publisher_statues(status_msg)
                    return False
                self.camera_active = True
                self.get_logger().info("摄像头已启动")
                return True
            except Exception as e:
                self.get_logger().error(f"启动摄像头时出错: {e}")
                return False
        return True

    def stop_camera(self):
        """停止摄像头"""
        if self.camera is not None:
            self.camera_active = False
            self.camera.release()
            self.camera = None
            self.get_logger().info("摄像头已停止")

    def camera_capture_thread(self):
        """摄像头捕获线程"""
        while rclpy.ok():
            if self.camera_active and self.camera is not None:
                ret, frame = self.camera.read()
                if ret:
                    self.current_frame = frame.copy()
            else:
                time.sleep(1)  # 降低CPU使用率



    def machinery_trigger_callback(self, msg):
        """接收到machinery_pub_image_trigger消息触发拍摄before图像"""
        if msg.data:
            time.sleep(1)
            print("sleep")
            self.capture_before_image()

    def person_trigger_callback(self, msg):
        """接收到person_pub_image_trigger消息触发拍摄after图像"""
        if msg.data:
            self.capture_after_image()

    def capture_before_image(self):
        """捕获移动前图像并发布"""
        if self.camera_active and self.current_frame is not None:
            self.before_image = self.current_frame.copy()
            print("已捕获移动前图像")
            self.publish_images()
        else:
            print("捕获移动前图像失败")

    def capture_after_image(self):
        """捕获移动后图像并发布"""
        if self.camera_active and self.current_frame is not None:
            self.after_image = self.current_frame.copy()
            print("已捕获移动后图像")
            self.publish_images()
        else:
            print("捕获移动后图像失败")

    def publish_images(self):
        """发布捕获的图像"""
        if self.before_image is not None:
            msg_before = self.bridge.cv2_to_imgmsg(self.before_image, encoding="bgr8")
            self.publisher_before.publish(msg_before)
            self.get_logger().info("已发布移动前图像")
            self.before_image=None
        if self.after_image is not None:
            msg_after = self.bridge.cv2_to_imgmsg(self.after_image, encoding="bgr8")
            self.publisher_after.publish(msg_after)
            self.get_logger().info("已发布移动后图像")
            self.after_image = None  # 确保使用空格缩进
	    
def main(args=None):
    rclpy.init(args=args)

    node = ChessCameraPublisher()

    try:
        # 主线程运行ROS节点
        rclpy.spin(node)  # ROS 2 节点保持运行，等待事件触发
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_camera()  # 停止摄像头
        cv2.destroyAllWindows()  # 销毁所有OpenCV窗口
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭ROS 2

if __name__ == '__main__':
    main()
