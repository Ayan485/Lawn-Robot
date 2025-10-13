import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriberGrayPublisher(Node):
    def __init__(self):
        super().__init__('grayscale_publisher')
        self.publisher_ = self.create_publisher(Image, 'grayscaleImg', 10)
        # self.timer = self.create_timer(0.1, self.gray_callback)
        # self.cap = cv2.VideoCapture(0)
        # if not self.cap.isOpened():
        #     self.get_logger().error('Could not open video stream.')
        #     exit()
        self.bridge = CvBridge()
        self.get_logger().info('grayscale publisher started.')
        self.subscription = self.create_subscription(Image, 'image_raw', self.gray_callback, 10)

    def gray_callback(self, msg):
        raw_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
        ros_gray = self.bridge.cv2_to_imgmsg(gray_image, 'mono8')
        self.publisher_.publish(ros_gray)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraSubscriberGrayPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()