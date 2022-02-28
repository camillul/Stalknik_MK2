
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
        super().__init__('simulation_node')

        # State variable
        self.x= float(1) 
        self.y= float(1) 
        self.z= float(0) 

        self.thetax = float(0) 
        self.thetay = float(0) 
        self.thetaz = float(0)

        self.x_dot= float(0) 
        self.y_dot= float(0) 
        self.z_dot= float(0) 

        self.thetax_dot = float(0) 
        self.thetay_dot = float(0) 
        self.thetaz_dot = float(0)



        # self.u1 = float(220) 
        # self.u2 = float(220) 
        # self.u3 = float(220)
        # self.u4 = float(220)
        # self.u1 = float(221.49) 
        # self.u2 = float(221.49) 
        # self.u3 = float(221.49)
        # self.u4 = float(221.49)
        self.u1 = float(0) 
        self.u2 = float(0) 
        self.u3 = float(0)
        self.u4 = float(0)


        self.speedLimit = 20
        self.rotLimit = math.pi
        # self.speedLimit = 100000
        # self.rotLimit = 100000

        self.m = 2 #kg
        self.arm_length = 0.5
        # coefficient from rotation (rad/s) to a force (N)
        self.b = 1
        # coeffiecient for counter rotation
        self.d = 1
        # (z_dot_dot,θx_dot_dot, θy_dot_dot, θz_dot_dot) = B*(u1,u2,u3,u4)
        self.B=np.array([[self.b,self.b,self.b,self.b],[-self.b*self.arm_length,0,self.b*self.arm_length,0],[0,-self.b*self.arm_length,0,self.b*self.arm_length],[-self.d,self.d,-self.d,self.d]]) 
        # Inertial matrix
        self.I=np.array([[10,0,0],[0,10,0],[0,0,20]])
        self.g = -9.81
   

        self.simulation_pub = self.create_publisher(Marker, 'drone_position', 10)
        # self.mapPub = self.create_publisher(Marker, 'map', 10)

        self.actuator_sub = self.create_subscription(
            Motor,
            'motor_mvmt',
            self.actuator_callback,
            10)

        self.actuator_sub  # prevent unused variable warning
        # time step
        self.dt = 0.1

        # self.tfPub = self.create_publisher(msg_type=TFMessage, topic="/tf", qos_profile = 10)
        # self.tf_static_Pub = self.create_publisher(msg_type=TFMessage, topic="/tf_static", qos_profile = 10)
        self._tf_publisher_static = StaticTransformBroadcaster(self)
        self._tf_publisher = TransformBroadcaster(self)
        self.make_transforms()




        self.timer = self.create_timer(1, self.simulation_callback)
        self.timer2 = self.create_timer(self.dt, self.dynamic)
        self.timer3 = self.create_timer(self.dt, self.make_transforms)

    def simulation_callback(self):

        # self.dynamic()

        # BUG:22/02/2022:Ricky:Can't show drone mesh and file is given with true path where it should be relative

        msg = Marker()
        msg.header.frame_id = "Stalknik"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.action = 0
        msg.type = 10
        msg.id = 1
        msg.mesh_resource = "file:///Users/rikic/Documents/Projet/Stalknik_MK2/ros_stalknik/src/simulation/resource/drone.dae"
        msg.color.a = 1.0
        msg.scale.x = 1.0
        msg.scale.y = 1.0
        msg.scale.z = 1.0

        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = float(self.z)
        q2 = quaternion_from_euler(self.thetax,self.thetay,self.thetaz)
        msg.pose.orientation.x = q2[0]
        msg.pose.orientation.y = q2[1]
        msg.pose.orientation.z = q2[2]
        msg.pose.orientation.w = q2[3]



        self.simulation_pub.publish(msg)
        # self.mapPub.publish(msg2)
        self.get_logger().info('Publishing simulation Position : "%s"' % msg.pose.position)

    def make_transforms(self):

        # TF side
        tf_msg = TFMessage()

        ts2_msg = TransformStamped()
        ts2_msg.header.stamp = self.get_clock().now().to_msg()
        ts2_msg.child_frame_id = "Stalknik"
        ts2_msg.header.frame_id = "map"
        ts2_msg.transform.translation.x = float(self.x)
        ts2_msg.transform.translation.y = float(self.y)
        ts2_msg.transform.translation.z = float(self.z)
        q2 = quaternion_from_euler(self.thetax,self.thetay,self.thetaz)
        ts2_msg.transform.rotation.x = q2[0]
        ts2_msg.transform.rotation.y = q2[1]
        ts2_msg.transform.rotation.z = q2[2]
        ts2_msg.transform.rotation.w = q2[3]

        self._tf_publisher.sendTransform(ts2_msg)



    def make_transforms_static(self):

        ts1_msg = TransformStamped()
        ts1_msg.header.stamp = self.get_clock().now().to_msg()
        ts2_msg.header.frame_id = "map"
        ts1_msg.transform.translation.x = float(0)
        ts1_msg.transform.translation.y = float(0)
        ts1_msg.transform.translation.z = float(0)
        q1 = quaternion_from_euler(float(0),float(0),float(0))
        ts1_msg.transform.rotation.x = q1[0]
        ts1_msg.transform.rotation.y = q1[1]
        ts1_msg.transform.rotation.z = q1[2]
        ts1_msg.transform.rotation.w = q1[3]

        # tf_msg.transforms.append(ts1_msg)
        # tf_msg.transforms.append(ts2_msg)

        self._tf_publisher_static.sendTransform(ts1_msg)
        # self.tfPub.publish(tf_msg)






    def actuator_callback(self,msg): 
        self.u1 = msg.m1
        self.u2 = msg.m2
        self.u3 = msg.m3
        self.u4 = msg.m4
        self.get_logger().info('I heard actuator command: ("%d","%d","%d","%d") ' %(msg.m1 ,msg.m2 ,msg.m3 ,msg.m4))




    def dynamic(self):
        self.state = np.array([[self.x,self.y,self.z, self.thetax, self.thetay, self.thetaz,self.x_dot,self.y_dot,self.z_dot, self.thetax_dot, self.thetay_dot, self.thetaz_dot]]).flatten()
        # FIXME:Ricky:26/02/2022: command must be adjust with * (-100) is it normal ? so if cmd is positive then z rise.
        w = -np.array([self.u1,self.u2,self.u3,self.u4])/100
        self.thetax,self.thetay,self.thetaz
        vr=(self.state[6:9]).reshape(3,1)
        wr=(self.state[9:12]).reshape(3,1)
        w2=w*abs(w)
        # torque is the torque felt by the entire system (!= motor torque), that's why we use w² 
        # v =  Eulermat * vdrone
        # a = v_dot = Eulermat_dot* vdrone + Eulermat*adrone
        torque=self.B@w2.flatten()
        E=eulermat(self.thetax,self.thetay,self.thetaz)
        dp=E@vr
        # la matrice adjointe est nécéssaire pour la formule axe-angle et en avoir une matrice de rotation (provient d'un produit vectoriel)
        dvr=-adjoint(wr)@vr+inv(E)@np.array([[0],[0],[self.g]])+np.array([[0],[0],[-torque[0]/self.m]])  
        dtheta= eulerderivative(self.thetax,self.thetay,self.thetaz) @ wr     
        dwr= inv(self.I)@(-adjoint(wr)@self.I@wr+torque[1:4].reshape(3,1)) 

        dstate = np.vstack((dp,dtheta,dvr,dwr)).flatten()
        self.state = self.state + dstate

        # self.get_logger().info('dstate : "%s"' %dstate)
        # self.get_logger().info('state : "%s"' %self.state)
        # self.get_logger().info('result : "%s"' %self.state)

        self.x= self.state[0]
        self.y= self.state[1]
        # z can't be negative because else it would mean the drone is under the ground
        # though the real limite is not 0 i depend on ground altitude
        if (self.state[2] < 0 ):
            self.state[2]= 0
            self.state[8] = 0
        self.z= self.state[2]
        self.thetax = self.state[3] 
        self.thetay = self.state[4]
        self.thetaz = self.state[5]
        # Physical limits implie acc and speed have a maximum (otherwise we would have an infinite energy sometime)
        self.x_dot= min(max(-self.speedLimit ,self.state[6]),self.speedLimit)
        self.y_dot= min(max(-self.speedLimit ,self.state[7]),self.speedLimit)  
        self.z_dot= min(max(-self.speedLimit ,self.state[8]),self.speedLimit)

        self.thetax_dot = min(max(-self.rotLimit ,self.state[9]),self.rotLimit)
        self.thetay_dot = min(max(-self.rotLimit ,self.state[10]),self.rotLimit)
        self.thetaz_dot = min(max(-self.rotLimit ,self.state[11]),self.rotLimit)




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