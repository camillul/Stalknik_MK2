import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import Pose
from drone_interfaces.srv import FollowUp

import math 
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # ****_sub and ****_pub are initialization for publisher and subscriber 
        # args : message type, "topic name", quality of service)
        self.drone_command_pub = self.create_publisher(Pose, 'drone_command', 10)
        self.car_position_sub = self.create_subscription(
            Pose,
            'car_position',
            self.car_position_callback,
            10)
        self.car_position_sub  # prevent unused variable warning
        self.drone_position_sub = self.create_subscription(
            Pose,
            'drone_position',
            self.drone_position_callback,
            10)
        self.drone_position_sub  # prevent unused variable warning

        # These are parameters for the "follow_up_callback" 
        self.follow_choice = "above"
        self.z_offset = 1.5
        self.distance_follow = 2
        # This init a server that will allow to create some request from the user and then change parameters
        self.srv = self.create_service(FollowUp, 'follow_up_service', self.follow_up_callback)   

        # timer is a timer/thread that call the callback function every t/timer_period/dt  seconds
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.drone_command_callback)
        self.i = float(0)



        # initialization of position and rotation array for our car and drone
        self.car_pos = np.array([0,0,0],np.float)
        self.car_orientation = np.array([0,0,0],np.float)
        self.drone_pos = np.array([0,0,0],np.float)
        self.drone_orientation = np.array([0,0,0],np.float)

    def follow_up_callback(self, request, response):
        if request.mode == 1 :
            self.follow_choice = "above"
            self.get_logger().info('Incoming request : change for above follow_up')
        if request.mode == 2 :
            self.follow_choice = "behind"
            self.get_logger().info('Incoming request : change for "spy" follow_up')
        if request.mode == 3 :
            self.follow_choice = "circle"
            self.get_logger().info('Incoming request : change for "spy" follow_up')

        response.result = "Done"

        return response


    def drone_command_callback(self):
        """
        This function is callback and raise at some event
        It will check which scenario has been chosen and then publish the according drone position/rotation aimed

        """

        drone_command_msg = Pose()
        
        if (self.follow_choice == "above") :
        # in this situation, we will just give a z offset and will follow the car on x,y world axis


            drone_command_msg.position.x = float(self.car_pos[0])
            drone_command_msg.position.y = float(self.car_pos[1])
            drone_command_msg.position.z = float(self.z_offset)

            self.drone_command_pub.publish(drone_command_msg)
            self.get_logger().info('Publishing above mode: "%s"' % drone_command_msg.position)

        if (self.follow_choice == "behind") :
        # In this situation, our drone will keep a maximum distance from the car and will keep "looking" toward that car
            distance = (self.car_pos - self.drone_pos)
            q_command = quaternion_from_euler(float(0),float(0),float(np.arctan2(distance[1],distance[0])))

            if   np.linalg.norm(distance)>=self.distance_follow:
                self.drone_command = self.car_pos + self.distance_follow * distance / np.linalg.norm(distance)
            else : 
                self.drone_command =  self.drone_pos

            drone_command_msg = Pose()

            print("from node control")
            print(float(self.drone_pos[0]))
            print(float(self.drone_pos[1]))

            drone_command_msg.position.x = float(self.drone_pos[0])
            drone_command_msg.position.y = float(self.drone_pos[1])
            drone_command_msg.position.z = float(self.drone_pos[2])

            drone_command_msg.orientation.x = q_command[0]
            drone_command_msg.orientation.y = q_command[1]
            drone_command_msg.orientation.z = q_command[2]
            drone_command_msg.orientation.w = q_command[3]

            self.drone_command_pub.publish(drone_command_msg)
            self.get_logger().info('Publishing behind mode : "%s"' % drone_command_msg.position)


        if (self.follow_choice == "circle") :
        # TODO:Ricky:27/02/2022:Need to implement parametrics curve such as circle around the car follow up
            pass



    def car_position_callback(self,msg):
        """
        This function is raised every time the associated topic receive a new message
        this function update our informations on the current (estimated) car position/orientation
        """
        self.get_logger().info('I received car_position: "%s"' % msg.position)
        self.car_pos = np.array([msg.position.x,msg.position.y,msg.position.z])
        self.car_orientation = np.array([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])



    def drone_position_callback(self,msg):
        """
        This function is raised every time the associated topic receive a new message
        this function update our informations on the current (estimated) drone position/orientation
        """
        self.get_logger().info('I received drone_position: "%s"' % msg.position)
        self.drone_pos = np.array([msg.position.x,msg.position.y,msg.position.z])
        self.drone_orientation = np.array([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])





def main(args=None):
    rclpy.init(args=args)
    Control = ControlNode()
    rclpy.spin(Control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()