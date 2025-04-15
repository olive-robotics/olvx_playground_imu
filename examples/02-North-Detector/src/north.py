import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import math

class MagnetometerNode(Node):
    def __init__(self):
        super().__init__('magnetometer_node')
        self.subscription = self.create_subscription(
            MagneticField,
            '/olive/imu/id001/magnetometer',
            self.magnetometer_callback,
            10)
        self.subscription  # prevent unused variable warning

    def magnetometer_callback(self, msg):
        # Calculate heading from magnetometer x and y data
        # This is a simple calculation and assumes that the magnetometer data is level with the ground
        heading = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
        
        # Convert heading to degrees
        heading_deg = heading * (180.0 / math.pi)

        # Magnetic declination correction (example value, should be set according to your location)
        declination = 0
        heading_deg += declination

        # Ensure heading is 0-360
        if heading_deg < 0:
            heading_deg += 360
        elif heading_deg > 360:
            heading_deg -= 360

        # Print heading
        self.get_logger().info('Heading: ' + str(heading_deg) + ' deg')

def main(args=None):
    rclpy.init(args=args)

    magnetometer_node = MagnetometerNode()

    try:
        rclpy.spin(magnetometer_node)
    except KeyboardInterrupt:
        pass  # allow Ctrl-C to end spin()

    magnetometer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
