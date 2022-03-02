import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from drone_interfaces.msg import Motor

pose = Pose();
class Actuator_node(Node):

    def __init__(self):
        super().__init__('actuator_node')
        self.publisher_ = self.create_publisher(Motor, 'motor_mvmt', 10)
        self.subscription = self.create_subscription(
            Pose,
            'drone_command',
            self.drone_position_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.actuator_command_callback)
        self.i = float(0)




    def actuator_command_callback(self):
    # TODO:05/02/2022:RODRIGUES:Implement this publisher
        msg = Motor()
        msg.m1 = float(300)
        msg.m2 = float(100)
        msg.m3 = float(300)
        msg.m4 = float(100)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1


    def drone_position_callback(self,msg):
    # TODO:05/02/2022:RODRIGUES:Implement listener 
        self.get_logger().info('I heard: "%s"' % msg.position)
        pose = msg





def main(args=None):
    rclpy.init(args=args)
    actuator = Actuator_node()
    rclpy.spin(actuator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actuator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()