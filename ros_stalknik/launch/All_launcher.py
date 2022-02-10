

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='img_processing',
            namespace='Stalknik',
            executable='node_img_processing',
            name='node_img_processing'
        ),
        Node(
            package='img_processing',
            namespace='Stalknik',
            executable='node_img_acquisition',
            name='node_img_acquisition'
        ),
        Node(
            package='drone_control',
            namespace='Stalknik',
            executable='node_control',
            name='node_control',
            
        ),
        Node(
            package='actuator_control',
            namespace='Stalknik',
            executable='node_actuator',
            name='node_actuator',
            
        )
    ])