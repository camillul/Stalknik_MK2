import time
import unittest
import sys

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import *

from std_msgs.msg import String
from geometry_msgs.msg import Pose

import pytest
sys.path.append("../")
from actuator_control.main import *

@pytest.mark.launch_test
# @launch_testing.markers.keep_alive
# def generate_test_description():
#     return launch.LaunchDescription([
#         launch.actions.TimerAction(
#             period=5.0,
#             actions=[
#                 launch_ros.actions.Node(
#                     executable='node_actuator',
#                     package='actuator_control',
#                     name='node_actuator'
#                 ),
#             ]),
#         launch_testing.actions.ReadyToTest()
#     ])

class test_launch(unittest.TestCase):

    def test_get_actuator_control(self):
        args=None
        rclpy.init(args=args)
        actuator = Actuator_node()
        rclpy.spin_once(actuator)        
        

        t1_init = Pose()
        t1_msg = Pose()

        t1_init.position.x = 0
        t1_init.position.y = 0
        t1_init.position.z = 0

        t1_msg.position.x = 0
        t1_msg.position.y = 0
        t1_msg.position.z = 10

        t1_u1, t1_u2, t1_u3, t1_u4 = Actuator_node.get_actuator_control()

        assert (t1_u1 <= 0 , t1_u2 <= 0 , t1_u3 <= 0 , t1_u4 <= 0)

        actuator_node.destroy_node()
        rclpy.shutdown()




    # TODO:Ricky:07/02/2022:More Test 

    def test2_get_actuator_control(self):
        """

        """
        pass

        


    
