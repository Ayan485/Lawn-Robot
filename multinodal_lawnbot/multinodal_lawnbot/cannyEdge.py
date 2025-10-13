import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdgePublisher(Node):
    def __init__(self):
        super().__init__('canny_publisher')
        self.publisher_ = self.create_publisher(Image, 'cannyImg', 10)
        self.bridge = CvBridge()
        self.get_logger().info('canny publisher started.')
        self.subscription = self.create_subscription(Image, 'grayscaleImg', self.canny_callback, 10)

    def canny_callback(self, msg):
        gray_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        canny_image = cv2.Canny(gray_image, 50,150)
        ros_canny = self.bridge.cv2_to_imgmsg(canny_image, 'mono8')
        self.publisher_.publish(ros_canny)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CannyEdgePublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()