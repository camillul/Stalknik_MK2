import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from core import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.imagePub = self.create_publisher(Image, 'image', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.im_list = []
        self.cv_image = cv2.imread('sample1.jpg') ### an RGB image 
        self.bridge = CvBridge()

    def img_callback(self):
        self.imagePub.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))
        self.get_logger().info('Publishing an image')
        
    

def main(args=None):
    rclpy.init(args=args)
    Img = ImgNode()
    rclpy.spin(Img)
    Img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()