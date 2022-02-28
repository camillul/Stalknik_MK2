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
from simulation.dynamic_sim import *
# import math
# from geometry_msgs.msg import Pose, TransformStamped
# from visualization_msgs.msg import Marker
# from drone_interfaces.msg import Motor
# from tf2_msgs.msg import TFMessage

# import tf2_ros
# from tf2_ros import TransformBroadcaster
# from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# from numpy.linalg import inv
# from scipy.linalg import expm
# import numpy as np

# @pytest.mark.launch_test
# @launch_testing.markers.keep_alive
# def generate_test_description():
#     return launch.LaunchDescription([
#         launch.actions.TimerAction(
#             period=5.0,
#             actions=[
#                 launch_ros.actions.Node(
#                     executable='node_simulation',
#                     package='simulation',
#                     name='node_simulation'
#                 ),
#             ]),
#         launch_testing.actions.ReadyToTest()
#     ])


class test_launch(unittest.TestCase):

    def test_sim_case_1(self):
        """
        In case 1, we will check that only z axis moved if :
        m1 = m2 = m3 = m4

        """
        args=None
        rclpy.init(args=args)
        Sim = SimulationNode()
        rclpy.spin_once(Sim)

        Sim.u1= 5
        Sim.u2= 5
        Sim.u3= 5
        Sim.u4= 5

        Sim.dynamic()

        time.sleep(1)
        assert (Sim.state[2] != 0)
        assert pytest.approx(Sim.state[2],0)
        assert pytest.approx(Sim.state[2],0)


        Sim.destroy_node()
        rclpy.shutdown()

        time.sleep(1)
    def test_sim_case_2(self):
        """
        In case 2, we will check that only z position and rotation axis moved if : :
        m1 = m3
        m2 = m4

        """
        args=None
        rclpy.init(args=args)
        Sim = SimulationNode()
        rclpy.spin_once(Sim)

        Sim.u1= 4
        Sim.u2= 1
        Sim.u3= 4
        Sim.u4= 1

        Sim.dynamic()
        time.sleep(1)
        assert (Sim.state[2] != 0  )
        assert pytest.approx(Sim.state[0],0)
        assert pytest.approx(Sim.state[1],0)

        assert (Sim.state[5] != 0)
        assert pytest.approx(Sim.state[3],0)
        assert pytest.approx(Sim.state[4],0)

        Sim.destroy_node()
        rclpy.shutdown()
        time.sleep(1)

    def test_sim_case_3(self):
        """
        In case 2, we will check that only z position and x rotation axis moved if : :
        m1 = m3
        m2 = m4

        """
        args=None
        rclpy.init(args=args)
        Sim = SimulationNode()
        rclpy.spin_once(Sim)

        Sim.u1= 3
        Sim.u2= 2
        Sim.u3= 1
        Sim.u4= 2

        Sim.dynamic()

        time.sleep(1)
        assert (Sim.state[2] != 0   )
        assert pytest.approx(Sim.state[0],0)
        assert pytest.approx(Sim.state[1],0)

        assert (Sim.state[3] != 0)
        assert pytest.approx(Sim.state[5],0)
        assert pytest.approx(Sim.state[4],0)

        Sim.destroy_node()
        rclpy.shutdown()
        time.sleep(1)

    def test_sim_case_4(self):
        """
        In case 2, we will check that only z position and y rotation axis moved if : :
        m1 = m3
        m2 = m4

        """
        args=None
        rclpy.init(args=args)
        Sim = SimulationNode()
        rclpy.spin_once(Sim)

        Sim.u1= 2
        Sim.u2= 3
        Sim.u3= 2
        Sim.u4= 1

        Sim.dynamic()
        
        time.sleep(1)
        print (Sim.state[2])
        assert (Sim.state[2] != 0  )
        assert pytest.approx(Sim.state[0],0)
        assert pytest.approx(Sim.state[1],0)

        assert (Sim.state[4] != 0)
        assert pytest.approx(Sim.state[5],0)
        assert pytest.approx(Sim.state[4],0)

        Sim.destroy_node()
        rclpy.shutdown()
        time.sleep(1)