
import rclpy
from rclpy.node import Node

import math
from geometry_msgs.msg import Pose, TransformStamped
from visualization_msgs.msg import Marker
from drone_interfaces.msg import Motor
from tf2_msgs.msg import TFMessage

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# import tf_conversions
# BUG::17/02/2022:Ricky: Can't import tf2 
# from tf2.transformations import quaternion_from_euler







from numpy.linalg import inv
from scipy.linalg import expm
import numpy as np


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

# 

def eulermat(thetax,thetay,thetaz):
    """
    euler matrix which is the product of 3 rotations matrix on x,y,z axis   
    """
    Ad_i = adjoint(np.array([1,0,0]))
    Ad_j = adjoint(np.array([0,1,0]))
    Ad_k = adjoint(np.array([0,0,1]))
    M = expm(thetaz*Ad_k) @ expm(thetay*Ad_j) @ expm(thetax*Ad_i)
    return(M)   

def eulerderivative(phi,theta,psi):
    """
    derative of the eurlermatrix
    """
    c_phi,s_phi,c_theta,s_theta,t_theta,c_psi,s_psi = math.cos(phi),math.sin(phi),math.cos(theta),math.sin(theta),math.sin(theta)/math.cos(theta),math.cos(psi),math.sin(psi)        
    return np.array([[1,s_phi*t_theta,c_phi*t_theta],[0, c_phi,-s_phi],[0,s_phi/c_theta,c_phi/c_theta]])   


def adjoint(w):
    """
    allow to give a matrix rotation form a vector w
    """    
    w=w.flatten()
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])


class SimulationNode(Node):



    def __init__(self):
        super().__init__('node_simulation')

        # Here we declare our parameters from YAML file
        self.declare_parameter('/Stalknik/actuator_control',False)
        self.actuator_control = self.get_parameter('/Stalknik/actuator_control').get_parameter_value()
        self.actuator_control = False
    
        self.declare_parameter('/Stalknik/my_dae_file','error')
        self.my_dae_file = self.get_parameter('/Stalknik/my_dae_file').get_parameter_value().string_value
        self.get_logger().info('dae param : "%s"' % self.my_dae_file)

        # step time for dynamic
        self.dt = 0.05

        # State variable
        # x, y, z are position variable respectively on x, y, z axis
        self.x= float(1) 
        self.y= float(1) 
        self.z= float(0) 
         # thetax, thetay, thetaz are rotation variable respectively on x, y, z axis
        self.thetax = float(0) 
        self.thetay = float(0) 
        self.thetaz = float(0)

        # _dot means derivative
        self.x_dot= float(0) 
        self.y_dot= float(0) 
        self.z_dot= float(0) 

        self.thetax_dot = float(0) 
        self.thetay_dot = float(0) 
        self.thetaz_dot = float(0)



        # Drone scheme and actuator position
        # ------------------------------------------------------------------
        # 
        #                      actuator_value_2
        #                              X
        #                              |
        #                              |
        #                              |
        #actuator_value_3   X--------<_._>--------X actuator_value_1
        #                              |
        #                              |
        #                              |
        #                              X
        # 
        #                       actuator_value_4
         # 
        #   Z    Y   
        #   |  7
        #   | /
        #   |/
        #    +----------->X
        # ---------------------------------------------------------------------


        self.actuator_value_1 = float(0) 
        self.actuator_value_2 = float(0) 
        self.actuator_value_3 = float(0)
        self.actuator_value_4 = float(0)

        #These are physic limit on speed and rotation (we can easly avoid then infinite energy)
        self.speedLimit = 20
        self.rotLimit = math.pi*10


        self.m = 2 #kg
        self.arm_length = 0.5
        # coefficient from rotation (rad/s) to a force (N)
        self.b = 1
        # coeffiecient for counter rotation
        self.d = 1
        # (z_dot_dot,θx_dot_dot, θy_dot_dot, θz_dot_dot) = B*(actuator_value_1,actuator_value_2,actuator_value_3,actuator_value_4)
        # then for instance thetax change on actuator_value_1 and actuator_value_3
        self.B=np.array([[self.b,self.b,self.b,self.b],[-self.b*self.arm_length,0,self.b*self.arm_length,0],[0,-self.b*self.arm_length,0,self.b*self.arm_length],[-self.d,self.d,-self.d,self.d]]) 
        # Inertial matrix
        self.I=np.array([[10,0,0],[0,10,0],[0,0,20]])
        self.g = -9.81

        self.simulation_pub = self.create_publisher(Marker, 'drone_position', 10)

        if self.actuator_control:
            self.actuator_sub = self.create_subscription(
                Motor,
                'motor_mvmt',
                self.actuator_callback,
                10)

            self.actuator_sub  # prevent unused variable warning
            
        else :  
            self.drone_command_sub = self.create_subscription(
                Pose,
                'drone_command',
                self.drone_command_callback,
                10)

            self.drone_command_sub  # prevent unused variable warning   
            self.drone_command_vec = np.array([0,0,0],np.float).flatten()

        
        

        # These are tf_publisher and will update transform/reference in RVIZ
        self._tf_publisher_static = StaticTransformBroadcaster(self)
        self._tf_publisher = TransformBroadcaster(self)
        self.make_transforms()

        # timer is a timer/thread that call the callback function every t/timer_period/dt  seconds
        self.timer = self.create_timer(1, self.simulation_callback)
        self.timer2 = self.create_timer(self.dt, self.dynamic)
        self.timer3 = self.create_timer(self.dt, self.make_transforms)

    def simulation_callback(self):
        """
        This function is callback and raise at some event
        It will publish a drone mesh to RVIZ and show his position and rotation

        """

        # BUG:22/02/2022:Ricky:Can't show drone mesh and file is given with true path where it should be relative



        marker_drone_rviz = Marker()
        marker_drone_rviz.header.frame_id = "Stalknik"
        marker_drone_rviz.header.stamp = self.get_clock().now().to_msg()
        marker_drone_rviz.action = 0
        marker_drone_rviz.type = 10
        marker_drone_rviz.id = 1
        marker_drone_rviz.mesh_resource = self.my_dae_file
        marker_drone_rviz.color.a = 1.0
        marker_drone_rviz.scale.x = 1.0
        marker_drone_rviz.scale.y = 1.0
        marker_drone_rviz.scale.z = 1.0

        marker_drone_rviz.pose.position.x = float(self.x)
        marker_drone_rviz.pose.position.y = float(self.y)
        marker_drone_rviz.pose.position.z = float(self.z)
        quaternion_drone = quaternion_from_euler(self.thetax,self.thetay,self.thetaz+np.pi)
        marker_drone_rviz.pose.orientation.x = quaternion_drone[0]
        marker_drone_rviz.pose.orientation.y = quaternion_drone[1]
        marker_drone_rviz.pose.orientation.z = quaternion_drone[2]
        marker_drone_rviz.pose.orientation.w = quaternion_drone[3]



        self.simulation_pub.publish(msg)
        self.get_logger().info('Publishing simulation Position : "%s"' % msg.pose.position)





    def make_transforms_static(self):
        """
        publish the map transform for rviz

        (only called once since it's a static paramater)
        """
        transform_map_msg = TransformStamped()
        transform_map_msg.header.stamp = self.get_clock().now().to_msg()
        transform_map_msg.header.frame_id = "map"
        transform_map_msg.transform.translation.x = float(0)
        transform_map_msg.transform.translation.y = float(0)
        transform_map_msg.transform.translation.z = float(0)
        quaternion_map = quaternion_from_euler(float(0),float(0),float(0))
        transform_map_msg.transform.rotation.x = quaternion_map[0]
        transform_map_msg.transform.rotation.y = quaternion_map[1]
        transform_map_msg.transform.rotation.z = quaternion_map[2]
        transform_map_msg.transform.rotation.w = quaternion_map[3]


        self._tf_publisher_static.sendTransform(transform_map_msg)




    def make_transforms(self):
        """
        publish the drone transform for rviz
        """
        
        tf_msg = TFMessage()
        transform_drone_msg = TransformStamped()
        transform_drone_msg.header.stamp = self.get_clock().now().to_msg()
        transform_drone_msg.child_frame_id = "Stalknik"
        transform_drone_msg.header.frame_id = "map"
        transform_drone_msg.transform.translation.x = float(self.x)
        transform_drone_msg.transform.translation.y = float(self.y)
        transform_drone_msg.transform.translation.z = float(self.z)
        quaternion_drone = quaternion_from_euler(self.thetaz,self.thetay,self.thetax)
        transform_drone_msg.transform.rotation.x = quaternion_drone[0]
        transform_drone_msg.transform.rotation.y = quaternion_drone[1]
        transform_drone_msg.transform.rotation.z = quaternion_drone[2]
        transform_drone_msg.transform.rotation.w = quaternion_drone[3]

        self._tf_publisher.sendTransform(transform_drone_msg)


    def actuator_callback(self,msg): 
        """
        This function is raised every time the associated topic receive a new message
        this function update our informations on the current actuator command 
        """
        self.actuator_value_1 = msg.m1
        self.actuator_value_2 = msg.m2
        self.actuator_value_3 = msg.m3
        self.actuator_value_4 = msg.m4
        self.get_logger().info('I heard actuator command: ("%d","%d","%d","%d") ' %(msg.m1 ,msg.m2 ,msg.m3 ,msg.m4))

    def drone_command_callback(self,msg):
        """
        This function is raised every time the associated topic receive a new message
        this function update our informations on the current drone position/rotation command 
        """
        self.get_logger().info('I heard control command: ("%d","%d","%d") ' %(msg.position.x ,msg.position.y ,msg.position.z))
        self.drone_command_vec[0] = msg.position.x 
        self.drone_command_vec[1] = msg.position.y 
        self.drone_command_vec[2] = msg.position.z        



    def dynamic(self):
    """
    This function will allow the drone to go from state t to state t+1 and so at a discrete accuracy gived by self.dt

    """
        if self.actuator_control:
            self.get_logger().info('Actuator dynamics')

            # self.state hold all drone state value (x,y,z), 3 angles and their derivation (position and speed)
            self.state = np.array([[self.x,self.y,self.z, self.thetax, self.thetay, self.thetaz,self.x_dot,self.y_dot,self.z_dot, self.thetax_dot, self.thetay_dot, self.thetaz_dot]]).flatten()
            # FIXME:Ricky:26/02/2022: command must be adjust with * (-100) is it normal ? so if cmd is positive then z rise.
            w = -np.array([self.actuator_value_1,self.actuator_value_2,self.actuator_value_3,self.actuator_value_4])/100
            self.thetax,self.thetay,self.thetaz
            # only speed
            vr=(self.state[6:9]).reshape(3,1)
            # only rotation speed
            wr=(self.state[9:12]).reshape(3,1)
            w2=w*abs(w)

            # torque is the torque felt by the entire system (!= motor torque), that's why we use w² 
            # v =  Eulermat * vdrone : formule de la vitesse 
            # a = v_dot = Eulermat_dot* vdrone + Eulermat * adrone : formule de l'accéleration dérivé de la première

            torque=self.B@w2.flatten()
            E=eulermat(self.thetax,self.thetay,self.thetaz)
            dp=E@vr
            # the adjoint matrix convert a vector to a matrix rotation
            dvr=-adjoint(wr)@vr+inv(E)@np.array([[0],[0],[self.g]])+np.array([[0],[0],[-torque[0]/self.m]])  
            dtheta= eulerderivative(self.thetax,self.thetay,self.thetaz) @ wr 
            
            dwr= inv(self.I)@(-adjoint(wr)@self.I@wr+torque[1:4].reshape(3,1)) 
            # self.dstate as the same format as self.state but with their dérivative (speed and acceleration)
            dstate = np.vstack((dp,dtheta,dvr,dwr)).flatten()
            # here we ass from stat t-1 to state t by adding dstate and so for each state variable we will have something like : x(t+1) = x(t) + dt*x_dot(t)
            self.state = self.state + self.dt*dstate

            # Physical limits implie acc and speed have a maximum (otherwise we would have an infinite energy sometime)
            self.state[6]= min(max(-self.speedLimit ,self.state[6]),self.speedLimit)
            self.state[7]= min(max(-self.speedLimit ,self.state[7]),self.speedLimit)  
            self.state[8]= min(max(-self.speedLimit ,self.state[8]),self.speedLimit)
            self.state[9] = min(max(-self.rotLimit ,self.state[9]),self.rotLimit)
            self.state[10] = min(max(-self.rotLimit ,self.state[10]),self.rotLimit)
            self.state[11] = min(max(-self.rotLimit ,self.state[11]),self.rotLimit)
            # z can't be negative because else it would mean the drone is under the ground
            # though the real limite is not 0 it will depend on the ground altitude
            if (self.state[2] < 0 ):
                self.state[2]= 0
                self.state[8] = 0       

            self.get_logger().info('dstate : "%s"' %dstate)
            self.get_logger().info('state : "%s"' %self.state)
            self.get_logger().info('result : "%s"' %self.state)


            # then assigning all state value to our Node variable (our memory)
            self.x= self.state[0]
            self.y= self.state[1]
            self.z= self.state[2]
            
            self.thetax = self.state[3] 
            self.thetay = self.state[4]
            self.thetaz = self.state[5]

            self.x_dot = self.state[6]
            self.y_dot = self.state[7]
            self.z_dot = self.state[8]

            self.thetax_dot = self.state[9] 
            self.thetay_dot = self.state[10]
            self.thetaz_dot = self.state[11]        
            
        else :

            self.x= self.drone_command_vec[0]
            self.y= self.drone_command_vec[1]
            self.z= self.drone_command_vec[2]





def main(args=None):
    rclpy.init(args=args)
    Sim = SimulationNode()
    rclpy.spin(Sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    Sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
