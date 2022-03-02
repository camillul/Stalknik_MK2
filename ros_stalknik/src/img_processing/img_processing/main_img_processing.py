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
      my_yolo_file = self.declare_parameter('my_yolo_file').get_parameter_value().string_value

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

      self.drone_pos_vec = np.array([[0,0,0]])

      self.vertical_angle_of_view = 41.4* np.pi /180
      self.horizontal_angle_of_view = 53.5* np.pi /180
      self.cam_angle = 20 * np.pi /180

      # Camera resolution
      self.resolution_x = 320
      self.resolution_y = 200

      # represent the distance between the drone and the lower pixels on the camera along x axis from drone reference
      # init a 0 and update each time from_xy_to_car_pos is called
      self.min_fov_dist_x = 0
      # represent the distance between the drone and the upper pixels on the camera along x axis from drone reference
      # init a 0 and update each time from_xy_to_car_pos is called
      self.max_fov_dist_x = 0

      # represent the distance between the drone and the most left pixels on the camera along y axis from drone reference
      # init a 0 and update each time from_xy_to_car_pos is called
      self.min_fov_dist_x = 0
      # represent the distance between the drone and the most right pixels on the camera along y axis from drone reference
      # init a 0 and update each time from_xy_to_car_pos is called
      self.max_fov_dist_x = 0

   def from_xy_to_car_pos(self,x,y)

      self.min_fov_dist_x = self.drone_pos_vec[2] / np.tan(cam_angle - horizontal_angle_of_view/2)
      self.max_fov_dist_x = self.drone_pos_vec[2] / np.tan(cam_angle + horizontal_angle_of_view/2)



      distance_x = min_fov_dist_x +  (x / self.resolution_x) * (self.max_fov_dist_x-self.min_fov_dist_x)


      y_projection_distance_norm = np.linalg.norm(np.array([[distance_x,0,self.drone_pos_vec[2]]]))


      A1 = y_projection_distance_norm * np.tan(horizontal_angle_of_view)
      distance_y = A1 * (y/self.resolution_y)

      distance_vec = np.array([[ distance_x, distance_y, -self.drone_pos_vec[2] ]])

      self.car_pos = self.drone_pos_vec + distance_vec

      return 

   def img_process_callback(self):
      msg = Pose()
      msg.position.x = -float(self.i)
      msg.position.y = float(self.i)
      self.i += 2
      self.carPositionPub.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.position)

   def img_callback(self,msg):

        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        # from frame, we can start post-processing
        self.get_logger().info("Image Received")


   def car_detection(self,img):
      detector = ObjectDetection()
      detector.setModelTypeAsYOLOv3()
      detector.setModelPath(my_yolo_file)
      detector.loadModel()
      returned_image, detections, extracted_objects = detector.detectObjectsFromImage(input_image=img,input_type="array",output_type="array",extract_detected_objects=True, minimum_percentage_probability=20)
      return   returned_image, detections, extracted_objects
      




def main(args=None):
    rclpy.init(args=args)
    ImgProcess = ImgProcessNode()
    rclpy.spin(ImgProcess)
    ImgProcess.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()