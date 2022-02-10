import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import *
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.imagePub = self.create_publisher(Image, 'image', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.img_callback)

        self.im_list = []
        self.cv_image = cv2.imread('C:/Users/rikic/Documents/Projet/Stalknik_MK2/ros_stalknik/src/img_processing/img_processing/sample1.png') 
        self.get_logger().info('Found test image')
        self.bridge = CvBridge()
        self.i = 0

    def img_callback(self):
        # self.imagePub.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))
        # self.get_logger().info('Publishing an image')
        frame = self.cv_image
        # processes image data and converts to ros 2 message
        msg = Image()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'ANI717'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()

        # publishes message
        self.imagePub.publish(msg)
        self.get_logger().info('%d Images Published' % self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    Img = ImgNode()
    rclpy.spin(Img)
    Img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()