import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SinusoidalPublisher(Node):
    def __init__(self, magnitude_pan, magnitude_tilt, period=1.0):
        super().__init__('sinusoidal_publisher')
        self.publisher_owl1pan = self.create_publisher(Float32, 'olive/servo/owl1pan/goal/position', 10)
        self.publisher_owl2pan = self.create_publisher(Float32, 'olive/servo/owl2pan/goal/position', 10)
        self.publisher_owl3pan = self.create_publisher(Float32, 'olive/servo/owl3pan/goal/position', 10)
        self.publisher_owl4pan = self.create_publisher(Float32, 'olive/servo/owl4pan/goal/position', 10)
        
        self.publisher_owl1tilt = self.create_publisher(Float32, 'olive/servo/owl1tilt/goal/position', 10)
        self.publisher_owl2tilt = self.create_publisher(Float32, 'olive/servo/owl2tilt/goal/position', 10)
        self.publisher_owl3tilt = self.create_publisher(Float32, 'olive/servo/owl3tilt/goal/position', 10)
        self.publisher_owl4tilt = self.create_publisher(Float32, 'olive/servo/owl4tilt/goal/position', 10)
        
        self.magnitude_pan = magnitude_pan
        self.magnitude_tilt = magnitude_tilt
        
        self.period = period
        self.time = 0.0

        # Ensure timer frequency accounts for the desired sinusoidal period
        timer_frequency = 0.01  # Frequency at which timer callback is triggered, in seconds
        self.timer = self.create_timer(timer_frequency, self.timer_callback)

    def timer_callback(self):
        msg_pan = Float32()
        msg_tilt = Float32()
        
        # Generate sinusoidal value
        msg_pan.data = self.magnitude_pan[0] + (self.magnitude_pan[1] - self.magnitude_pan[0]) * (math.sin(self.time / self.period) * 0.5 + 0.5)
        msg_tilt.data = self.magnitude_tilt[0] + (self.magnitude_tilt[1] - self.magnitude_tilt[0]) * (math.sin(self.time / self.period) * 0.5 + 0.5)
        
        self.publisher_owl1pan.publish(msg_pan)
        self.publisher_owl2pan.publish(msg_pan)
        self.publisher_owl3pan.publish(msg_pan)
        self.publisher_owl4pan.publish(msg_pan)
        self.get_logger().info('Publishing pan: "%s"' % msg_pan.data)
        
        self.publisher_owl1tilt.publish(msg_tilt)
        self.publisher_owl2tilt.publish(msg_tilt)
        self.publisher_owl3tilt.publish(msg_tilt)
        self.publisher_owl4tilt.publish(msg_tilt)
        self.get_logger().info('Publishing tilt: "%s"' % msg_tilt.data)

        self.time += 0.01  # Increment time

def main(args=None):
    rclpy.init(args=args)
    
    # Customize magnitude and period as needed
    magnitude_pan = [-math.pi * 0.5, math.pi * 0.5]  # Amplitude of the sinusoid
    magnitude_tilt = [-1.3,0.9]  # Amplitude of the sinusoid
    period = 0.75  # Period of the sinusoid in seconds
    sinusoidal_publisher = SinusoidalPublisher(magnitude_pan, magnitude_tilt, period)

    try:
        rclpy.spin(sinusoidal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sinusoidal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

