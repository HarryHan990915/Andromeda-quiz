from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_dir = os.path.join(os.getenv('COLCON_PREFIX_PATH').split(':')[0], 'andromeda_proximity_filter')

    return LaunchDescription([
        # Launch your Python test publisher
        Node(
            package='andromeda_proximity_filter',
            executable='test_pointcloud_pub.py',
            output='screen'
        ),

        # Launch the proximity filter node
        Node(
            package='andromeda_proximity_filter',
            executable='proximity_filter',
            name='proximity_filter',
            output='screen',
            parameters=[{
                'polygon_x': [0.5, 0.25, -0.25, -0.5, -0.25, 0.25],
                'polygon_y': [0.0, 0.433, 0.433, 0.0, -0.433, -0.433],
                'max_range': 1.5,
                'fov_center_deg': 0.0,
                'fov_width_deg': 180.0,
                'input_topic': '/points/raw',
                'output_topic': '/points/filtered',
                'polygon_topic': '/points/filter_polygon',
                'subscription_rate': 10
            }]
        )
    ])
