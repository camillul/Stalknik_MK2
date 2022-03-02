import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    params = os.path.join(
        get_package_share_directory('img_processing'),
        'param',
        'params.yaml'
        )
        
    node_1= Node(
    package='img_processing',
    namespace='Stalknik',
    executable='node_img_processing',
    name='node_img_processing',
    parameters=[params]
              )
    
    node_2 = Node(
    package='img_processing',
    namespace='Stalknik',
    executable='node_img_acquisition',
    name='node_img_acquisition',
    parameters=[params]
              )

    ld.add_action(node_1)
    ld.add_action(node_2)


    return ld

