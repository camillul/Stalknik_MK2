import os
from glob import glob
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    launch_include_actuator_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('actuator_control'),
                'launch','local.launch.py') )
    )

    launch_include_drone_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drone_control'),
                'launch','local.launch.py') )
    )

    launch_include_img_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('img_processing'),
                'launch','local.launch.py') )
    )

    launch_include_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('simulation'),
                'launch','local.launch.py') )
    )



    return LaunchDescription([

        launch_include_actuator_control,
        launch_include_drone_control,
        launch_include_img_processing,
        launch_include_simulation
    ])