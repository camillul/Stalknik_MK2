import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
import numpy as np

class ImgProcessNode(Node):
   def __init__(self):
      super().__init__('img_process_node')
      self.carPositionPub = self.create_publisher(Pose, 'car_position', 10)
      timer_period = 0.5  # seconds
      self.timer = self.create_timer(timer_period, self.img_process_callback)
      self.im_list = []

      self.imageSub = self.create_subscription(
            Image,
            'image',
            self.img_callback,
            10)
      self.imageSub  # prevent unused variable warning

      self.bridge = CvBridge()

      self.i = 0



   def img_process_callback(self):
      msg = Pose()
      msg.position.x = -float(self.i)
      msg.position.y = float(self.i)
      self.i += 2
      self.carPositionPub_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.position)
      self.get_logger().info('Publishing an image')

   def img_callback(self,msg):
      self.get_logger().info('I received an : encoding "%s" "%d" , "%d" image format' % msg.encoding, msg.height, msg.width)

def main(args=None):
    rclpy.init(args=args)
    ImgProcess = ImgProcessNode()
    rclpy.spin(ImgProcess)
    ImgProcess.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()