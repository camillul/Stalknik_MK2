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
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    executable='node_actuator',
                    package='actuator_control',
                    name='node_actuator'
                ),
            ]),
        launch_testing.actions.ReadyToTest()
    ])

# Test written by Ricky 

class TestFixture(unittest.TestCase):

    def test1_get_actuator_control(self):
        """

        In this test, we are checking that all motors give positive value 


        """


        rclpy.init()
        t1_actuator = Actuator_node()
        # rclpy.spin(t1_actuator)        
        

        t1_current_pos = Pose()
        t1_msg = Pose()

        t1_current_pos.position.x = 0
        t1_current_pos.position.y = 0
        t1_current_pos.position.z = 0


        t1_msg.position.x = 0
        t1_msg.position.y = 0
        t1_msg.position.z = 10

        t1_u1, t1_u2, t1_u3, t1_u4 = t1_actuato.get_actuator_control(t1_msg)

        assert (t1_u1 <= 0 & t1_u2 <= 0 & t1_u3 <= 0 & t1_u4 <= 0)

        actuator_node.destroy_node()
        rclpy.shutdown()


    # TODO:Ricky:07/02/2022:More Test 

    def test2_get_actuator_control(self):
        """

        """
        pass



    
