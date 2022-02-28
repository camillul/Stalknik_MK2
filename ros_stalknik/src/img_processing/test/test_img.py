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

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import pytest

sys.path.append("../")

from img_processing.main_img_processing import *
from img_processing.main_img import *

@pytest.mark.launch_test
@launch_testing.markers.keep_alive

def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    executable='node_img_processing',
                    package='img_processing',
                    name='node_img_processing'
                ),
            ]),
        launch_testing.actions.ReadyToTest()
    ])

# Test written by Ricky 

class TestFixture(unittest.TestCase):

    def test1_image_detection(self):
        """

        In this test, we are checking that the right object is detected

        """


        rclpy.init()
        t1_img_processing = ImgProcessNode()
        # rclpy.spin(t1_actuator)        
        t1_raw_img =  cv2.imread('C:/Users/rikic/Documents/Projet/Stalknik_MK2/ros_stalknik/src/img_processing/test/sample3.jpg') 
        returned_image, detections, extracted_objects = t1_img_processing.car_detection(t1_raw_img)
        cv2.imshow('result t1_img',returned_image)
        cv2.waitKey(0) 

        t1_img_processing.destroy_node()
        rclpy.shutdown()


    # TODO:Ricky:07/02/2022:More Test 





    
