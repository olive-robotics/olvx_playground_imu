import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import math
import random

class Demo(Node):
    def __init__(self):
        super().__init__('demo')

        # Customize magnitude and period as needed
        self.sin_magnitude_pan = [-math.pi * 0.5, math.pi * 0.5]  # Amplitude of the sinusoid
        self.sin_magnitude_tilt = [-0.5,0.1]  # Amplitude of the sinusoid
        self.ran_magnitude_pan = [-math.pi * 0.5, math.pi * 0.5]  # Amplitude of the sinusoid
        self.ran_magnitude_tilt = [-0.8,0.3]  # Amplitude of the sinusoid
        self.period_pan = 1.5  # Period of the sinusoid in seconds
        self.period_tilt = 0.375

        #ROS2
        self.publisher_owl1pan = self.create_publisher(Float32, 'olive/servo/owl1pan/goal/position', 100)
        self.publisher_owl2pan = self.create_publisher(Float32, 'olive/servo/owl4pan/goal/position', 100)
        self.publisher_owl3pan = self.create_publisher(Float32, 'olive/servo/owl3pan/goal/position', 100)  
        self.publisher_owl1tilt = self.create_publisher(Float32, 'olive/servo/owl1tilt/goal/position', 100)
        self.publisher_owl2tilt = self.create_publisher(Float32, 'olive/servo/owl4tilt/goal/position', 100)
        self.publisher_owl3tilt = self.create_publisher(Float32, 'olive/servo/owl3tilt/goal/position', 100)
        self.sub_imu = self.create_subscription(Imu, 'olive/imu/one/imu', self.callback_imu, 10)
       
        self.imu_pan = 0.0
        self.imu_tilt = 0.0

        #Messages
        self.msg_pan_1 = Float32()
        self.msg_tilt_1 = Float32()
        self.msg_pan_2 = Float32()
        self.msg_tilt_2 = Float32()
        self.msg_pan_3 = Float32()
        self.msg_tilt_3 = Float32()
        
        #Modes
        self.mode = 3 #1 sin #2 random #3 imu
        self.time_index = 0
        self.mode_1_step_index = 0
        self.mode_2_step_index = 0
        self.mode_3_step_index = 0
        self.time = 0.0

        timer_frequency = 0.1  # Frequency at which timer callback is triggered, in seconds
        self.timer = self.create_timer(timer_frequency, self.timer_callback)

    def callback_imu(self, msg):

        # self.get_logger().info('Get imu')

        quat_msg = msg.orientation

        quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        rot = R.from_quat(quat)
        roll, pitch, yaw = rot.as_euler('xyz')

        self.imu_pan = -1 * yaw
        self.imu_tilt = -1 * pitch

    def timer_callback(self):

        if self.time_index > 30: # each segment is 30 seconds
            self.mode += 1
            self.time_index = 0
        
        if self.mode > 3:
            self.mode = 1

        if self.mode == 1: #sin

            value_pan = self.sin_magnitude_pan[0] + (self.sin_magnitude_pan[1] - self.sin_magnitude_pan[0]) * (math.sin(self.time / self.period_pan) * 0.5 + 0.5)
            value_tilt = self.sin_magnitude_tilt[0] + (self.sin_magnitude_tilt[1] - self.sin_magnitude_tilt[0]) * (math.sin(self.time / self.period_tilt) * 0.5 + 0.5)

            self.msg_pan_1.data = value_pan
            self.msg_tilt_1.data = value_tilt
            self.msg_pan_2.data = value_pan
            self.msg_tilt_2.data = value_tilt
            self.msg_pan_3.data = value_pan
            self.msg_tilt_3.data = value_tilt
            self.time += 0.1  # Increment time

        if self.mode == 2: #random

            self.mode_2_step_index += 1

            if self.mode_2_step_index == 10: # second 1

                self.msg_pan_1.data = random.uniform(self.ran_magnitude_pan[0], self.ran_magnitude_pan[1])
                self.msg_pan_2.data = random.uniform(self.ran_magnitude_pan[0], self.ran_magnitude_pan[1])
                self.msg_pan_3.data = random.uniform(self.ran_magnitude_pan[0], self.ran_magnitude_pan[1])

            if self.mode_2_step_index == 20: # second 2

                self.mode_2_step_index = 0
                self.msg_tilt_1.data = random.uniform(self.ran_magnitude_tilt[0], self.ran_magnitude_tilt[1])
                self.msg_tilt_2.data = random.uniform(self.ran_magnitude_tilt[0], self.ran_magnitude_tilt[1])
                self.msg_tilt_3.data = random.uniform(self.ran_magnitude_tilt[0], self.ran_magnitude_tilt[1])

        if self.mode == 3: #imu

            self.msg_pan_1.data = self.imu_pan
            self.msg_tilt_1.data = self.imu_tilt
            self.msg_pan_2.data = self.imu_pan
            self.msg_tilt_2.data = self.imu_tilt
            self.msg_pan_3.data = self.imu_pan
            self.msg_tilt_3.data = self.imu_tilt

        self.publisher_owl1pan.publish(self.msg_pan_1)
        self.publisher_owl2pan.publish(self.msg_pan_2)
        self.publisher_owl3pan.publish(self.msg_pan_3)

        #self.get_logger().info('Publishing pan 1: "%s"' % self.msg_pan_1.data)
        #self.get_logger().info('Publishing pan 2: "%s"' % self.msg_pan_2.data)
        #self.get_logger().info('Publishing pan 3: "%s"' % self.msg_pan_3.data)
        
        self.publisher_owl1tilt.publish(self.msg_tilt_1)
        self.publisher_owl2tilt.publish(self.msg_tilt_2)
        self.publisher_owl3tilt.publish(self.msg_tilt_3)

        #self.get_logger().info('Publishing tilt 1: "%s"' % self.msg_tilt_1.data)
        #self.get_logger().info('Publishing tilt 2: "%s"' % self.msg_tilt_2.data)
        #self.get_logger().info('Publishing tilt 3: "%s"' % self.msg_tilt_3.data)

        self.time_index += 0.1

        self.get_logger().info('Mode : "%s"' % self.mode)
    
def main(args=None):
    rclpy.init(args=args)
    
    demo_publisher = Demo()

    try:
        rclpy.spin(demo_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        demo_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

