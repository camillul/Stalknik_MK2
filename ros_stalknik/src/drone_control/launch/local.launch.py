import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    params = os.path.join(
        get_package_share_directory('drone_control'),
        'param',
        'params.yaml'
        )
        
    node= Node(
    package='drone_control',
    namespace='Stalknik',
    executable='node_control',
    name='node_control',
    parameters=[params]
              )

    ld.add_action(node)
    return ld