import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
import numpy as np


from imageai.Detection import ObjectDetection

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
      self.carPositionPub.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.position)

   def img_callback(self,msg):
    #   self.get_logger().info('I received an : encoding "%s" "%d" , "%d" image format' % msg.encoding, msg.height, msg.width)
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        # from frame, we can start post-processing
        self.get_logger().info("Image Received")


   def car_detection(self,img):
      detector = ObjectDetection()
      detector.setModelTypeAsYOLOv3()
      detector.setModelPath("C:/Users/rikic/Documents/Projet/Stalknik_MK2/ros_stalknik/src/img_processing/resource/yolo.h5")
      detector.loadModel()
      returned_image, detections, extracted_objects = detector.detectObjectsFromImage(input_image=img,input_type="array",output_type="array",extract_detected_objects=True, minimum_percentage_probability=20)
      # detections, extracted_objects = detector.detectObjectsFromImage(input_image=img,input_type="array" ,output_type="array",output_image_path="res1.jpg",extract_detected_objects=True, minimum_percentage_probability=30)
      return   returned_image, detections, extracted_objects




def main(args=None):
    rclpy.init(args=args)
    ImgProcess = ImgProcessNode()
    rclpy.spin(ImgProcess)
    ImgProcess.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()