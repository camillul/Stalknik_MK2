import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Pose

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.droneCommandPub_ = self.create_publisher(Pose, 'drone_command', 10)
        self.carPositionSub = self.create_subscription(
            Pose,
            'car_position',
            self.car_position_callback,
            10)
        self.carPositionSub  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.drone_command_callback)
        self.i = float(0)




    def drone_command_callback(self):
    # TODO:01/02/2022:CAMILLE:Implement this publisher
        msg = Pose()
        msg.position.z = float(10)
        msg.position.x = -float(self.i)
        msg.position.y = float(self.i)
        self.i += 2
        self.droneCommandPub_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)
        self.i += 2



    def car_position_callback(self,msg):
    # TODO:01/02/2022:CAMILLE:Implement listener 
        self.get_logger().info('I received: "%s"' % msg.position)




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