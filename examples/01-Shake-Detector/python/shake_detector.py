import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

# CONFIG
SUBSCRIPTION_TOPIC = '/olive/imu/id001/filtered_imu'
PUBLISHING_TOPIC = '/olive/app/shake_detection'

class AppNode(Node):

    def __init__(self):
        super().__init__('app_node')
        self.publisher_ = self.create_publisher(String, PUBLISHING_TOPIC, 1)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Imu,
            SUBSCRIPTION_TOPIC,
            self.listener_callback,
            1)
        
        self.subscription  # prevent unused variable warning
        self.acc = Vector3()
        self.acc_v = 0
        self.acc_thershold = 10

    def timer_callback(self):
        msg = String()
        self.shake = "false"
        
        self.acc_v = math.sqrt(self.acc.x * self.acc.x + self.acc.y * self.acc.y + self.acc.z * self.acc.z)   
        
        if self.acc_v > self.acc_thershold:
            self.shake = "true"
        else:
            self.shake = "false"
        
        msg.data = 'Shaking: %s' % self.shake
        self.publisher_.publish(msg)
        
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
    def listener_callback(self, msg):
        self.acc = msg.linear_acceleration
        
        
def main(args=None):
    rclpy.init(args=args)
    app_node = AppNode()
    rclpy.spin(app_node)
    
    app_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
