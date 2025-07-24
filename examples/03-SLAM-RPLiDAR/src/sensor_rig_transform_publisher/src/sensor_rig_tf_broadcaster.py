#!/usr/bin/env python3
import math
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster  # Changed to dynamic broadcaster

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    Returns (qx, qy, qz, qw)
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return (qx, qy, qz, qw)

class MultiTFBroadcaster(Node):
    def __init__(self):
        super().__init__('sensor_rig_dynamic_transform_publisher')

        # Declare a parameter for the YAML file path
        self.declare_parameter('yaml_path', '')

        # Create a dynamic transform broadcaster (publishes to /tf)
        self._broadcaster = TransformBroadcaster(self)

        # Read the initial param value and load/publish transforms
        yaml_file_path = self.get_parameter('yaml_path').get_parameter_value().string_value
        if yaml_file_path:
            self.load_and_publish_from_yaml(yaml_file_path)
        else:
            self.get_logger().warn("No 'yaml_path' param provided. Transforms will not be published until set.")

    def load_and_publish_from_yaml(self, path):
        """
        Reads the transforms from the given YAML file and continuously publishes them.
        """
        if not os.path.isfile(path):
            self.get_logger().error(f"YAML file not found: {path}")
            return

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to parse YAML file: {e}")
            return

        transforms_data = data.get('transforms', [])
        if not isinstance(transforms_data, list):
            self.get_logger().error("Invalid YAML format: 'transforms' key must be a list.")
            return

        self.transforms = []

        for idx, tf_item in enumerate(transforms_data):
            try:
                parent = tf_item['parent_frame']
                child = tf_item['child_frame']
                x = float(tf_item.get('x', 0.0))
                y = float(tf_item.get('y', 0.0))
                z = float(tf_item.get('z', 0.0))
                roll = float(tf_item.get('roll', 0.0))
                pitch = float(tf_item.get('pitch', 0.0))
                yaw = float(tf_item.get('yaw', 0.0))

                (qx, qy, qz, qw) = euler_to_quaternion(roll, pitch, yaw)

                t = TransformStamped()
                t.header.frame_id = parent
                t.child_frame_id = child
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw

                self.transforms.append(t)

            except KeyError as e:
                self.get_logger().error(f"Missing key in transform entry {idx}: {e}")
            except Exception as e:
                self.get_logger().error(f"Error parsing transform entry {idx}: {e}")

        # Start a timer to continuously publish transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)  # Publish at 10 Hz
        self.get_logger().info(f"Loaded {len(self.transforms)} transforms from {path}")

    def publish_transforms(self):
        """ Continuously publishes the transforms at a fixed rate """
        if self.transforms:
            for t in self.transforms:
                t.header.stamp = self.get_clock().now().to_msg()
            self._broadcaster.sendTransform(self.transforms)


def main(args=None):
    rclpy.init(args=args)
    node = MultiTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
