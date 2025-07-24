import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('sensor_rig_transform_publisher')
    yaml_path = os.path.join(pkg_dir, 'config', 'transforms.yaml')

    broadcaster_node = Node(
        package='sensor_rig_transform_publisher',
        executable='sensor_rig_tf_broadcaster.py',
        name='sensor_rig_transform_publisher_node',
        output='screen',
        parameters=[{'yaml_path': yaml_path}],
    )

    return LaunchDescription([broadcaster_node])
