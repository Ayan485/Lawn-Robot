import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

class Simple_service(Node):
    def __init__(self):
        super().__init__('float_service')
        self.get_logger().info('Multiplier service started.')
        self.subscription = self.create_subscription(Float64, 'Lawn', self.text_callback, 10)
        self.srv = self.create_service(Trigger,'Multiplied_by_90', self.service_callback)

    def text_callback(self, msg):
        self.latest=msg.data

    def service_callback(self,request, response):
        result = self.latest*90
        response.success = True
        response.message = f"the multiplied result is {result}"
        return response

def main(args=None):
    rclpy.init(args=args)
    float_service = Simple_service()
    rclpy.spin(float_service)
    float_service.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()