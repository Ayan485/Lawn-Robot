import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Trigger

class CannyEdgeService(Node):
    def __init__(self):
        super().__init__('canny_service')
        self.bridge = CvBridge()
        self.get_logger().info('canny subscriber started.')
        self.subscription = self.create_subscription(Image, 'image_raw', self.canny_callback, 10)
        self.srv = self.create_service(Trigger, "Canny_Service", self.canny_service)
        self.latest = None

    def canny_callback(self, msg):
        raw_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
        canny_image = cv2.Canny(gray_image, 50,150)
        self.latest = canny_image

    def canny_service(self, request, response):
        if self.latest is None:
            response.success = False
            response.message = "No image available yet."
            return response

        try:
            result = self.latest
            cv2.imshow("Canny Edge Detection", result)
            cv2.waitKey(0)

            response.success = True
            response.message = "Canny image displayed."

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response


def main(args=None):
    rclpy.init(args=args)
    canny_service = CannyEdgeService()
    rclpy.spin(canny_service)
    canny_service.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()