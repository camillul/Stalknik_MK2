import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
pose = Pose();
class Actuator_node(Node):

    def __init__(self):
        super().__init__('actuator_node')
        self.publisher_ = self.create_publisher(String, 'motor_mvmt', 10)
        self.subscription = self.create_subscription(
            Pose,
            'drone_position',
            self.drone_position_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.actuator_command_callback)
        
        self.i = float(0)




    def actuator_command_callback(self):
    # TODO:05/02/2022:RODRIGUES:Implement this publisher
        msg = String()
        msg.data = 'Go to: %d, %d, %d' % (pose.position.x, pose.position.y, pose.position.z)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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