import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
# from cv_bridge import CvBridge
import numpy as np

import time
import cv2
import numpy
import torch
import sys

from imageai.Detection import ObjectDetection




def mapping(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


class ImgProcessNode(Node):
   def __init__(self):
      super().__init__('img_process_node')

      self.declare_parameter('/Stalknik/my_yolo_file','error')
      self.my_yolo_file = self.get_parameter('/Stalknik/my_yolo_file').get_parameter_value().string_value

      self.declare_parameter('/Stalknik/my_trained_model','C:/Users/rikic/Documents/Projet/Stalknik_MK2/Cozmo_follow_car/trained_carmodel/car_cozmo.pt')
      self.my_trained_model = self.get_parameter('/Stalknik/my_trained_model').get_parameter_value().string_value

      self.declare_parameter('/Stalknik/IsCustom',True)
      self.IsCustom = self.get_parameter('/Stalknik/IsCustom').get_parameter_value()

      self.car_position_pub = self.create_publisher(Pose, 'car_position', 10)


 

      self.image_sub = self.create_subscription(
            Image,
            'image',
            self.img_callback, qos_profile_sensor_data  )
      self.image_sub  # prevent unused variable warning

      self.img_result = None
      self.latest_processed_image = None
      self.new_image_to_process = None
      self.finish_process = False
      

      self.car_pos_vec =  np.array([0,0,0])
      self.drone_pos_vec = np.array([0,0,0])

      self.vertical_angle_of_view = 41.4* np.pi /180
      self.horizontal_angle_of_view = 53.5* np.pi /180
      self.cam_angle = 20 * np.pi /180

      # Camera resolution
      self.resolution_x = 320
      self.resolution_y = 240

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


      # image processing variable

      # may need to use at least once : force_reload = True
      self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.my_trained_model)
      
      timer_period = 1  # seconds
      self.timer = self.create_timer(timer_period, self.img_process_callback)
      


   def imgmsg_to_cv2(self, img_msg, encode = "mono8"):
      if img_msg.encoding != encode:
         self.get_logger().info("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
      dtype = np.dtype("uint8") # Hardcode to 8 bits...
      dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
      image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                     dtype=dtype, buffer=img_msg.data)
      # If the byt order is different between the message and the system.
      if img_msg.is_bigendian == (sys.byteorder == 'little'):
         image_opencv = image_opencv.byteswap().newbyteorder()
      return image_opencv


   def from_xy_to_car_pos(self,x,y):
      """
      this function give x,y position of the car for a drone, this one must be above 
      
      """
      # FIXME:Ricky:14/02/2022: Don't work ahah and work only for a drone with z > 0 and a cam not aiming straigth downside

      self.min_fov_dist_x = self.drone_pos_vec[2] / np.tan(self.cam_angle - self.horizontal_angle_of_view/2)
      self.max_fov_dist_x = self.drone_pos_vec[2] / np.tan(self.cam_angle + self.horizontal_angle_of_view/2)



      distance_x = self.min_fov_dist_x +  (x / self.resolution_x) * (self.max_fov_dist_x-self.min_fov_dist_x)


      y_projection_distance_norm = np.linalg.norm(np.array([[distance_x,0,self.drone_pos_vec[2]]]))


      A1 = y_projection_distance_norm * np.tan(self.horizontal_angle_of_view)
      distance_y = A1 * (y/self.resolution_y)

      distance_vec = np.array([[ distance_x, distance_y, -self.drone_pos_vec[2] ]])

      self.car_pos_vec = self.drone_pos_vec + distance_vec

      

   def img_process_callback(self):
      
      self.get_logger().info('img_callback')
      self.finish_process = False
      if self.IsCustom :
         self.get_logger().info('Custom processing')
         self.custom_car_detection(self.new_image_to_process)


      else :
         self.car_detection(self.new_image_to_process)
         # while(not self.finish_process):
         #    pass

      
      if (self.finish_process == True) : 

         # self.latest_processed_image = self.new_image_to_process
         msg = Pose()
         vec = self.car_pos_vec.flatten()
         msg.position.x = float(vec[0])
         msg.position.y = float(vec[1])
         msg.position.z = float(vec[2])

         # TODO:Ricky:04/03/2022: Implement rotation detection

         self.car_position_pub.publish(msg)
         self.get_logger().info('Publishing : "%s"' % msg.position)

         cv2.imshow('img_processing', self.img_result)

         
         cv2.waitKey(33) == 27
            
         







   def img_callback(self,msg):

      self.get_logger().info("sub callback")
      
      # self.new_image_to_process = self.imgmsg_to_cv2(msg)
      try:
         self.new_image_to_process = self.imgmsg_to_cv2(msg)
         # self.new_image_to_process = np.array(self.frame, dtype=np.uint8)
         self.get_logger().info("Image Received")
         if self.new_image_to_process is not None:
            cv2.imshow('Received by img_processing', self.new_image_to_process)
            cv2.waitKey(1)
      except: 
         pass


      # Convert the image to a Numpy array since most cv2 functions
      # require Numpy arrays.
      

   def car_detection(self,img):
      detector = ObjectDetection()
      detector.setModelTypeAsYOLOv3()
      detector.setModelPath(self.my_yolo_file)
      detector.loadModel()
      returned_image, detections, extracted_objects = detector.detectObjectsFromImage(input_image=img,input_type="array",output_type="array",extract_detected_objects=True, minimum_percentage_probability=20)
      
      self.finish_process = True
      
   def custom_car_detection(self,img):

      self.get_logger().info('Custom detection if')
      if self.new_image_to_process is not None:
         self.get_logger().info('Custom detection begin')
         #OPENCV IMAGE SHOW
         im = self.new_image_to_process
         open_cv_image = im

         detection = self.model(open_cv_image, size=320)  # includes NMS
         self.get_logger().info('Custom detection detection')
         results_numpy = detection.pred[0].cpu().detach().numpy()
         
         for result in results_numpy:
            xa, ya, xb, yb, conf, cls = result
            if conf > 0.3:
               x1, y1, x2, y2 = [round(f) for f in [xa, ya, xb, yb]]

            cv2.rectangle(open_cv_image,(x1,y1),(x2,y2),(0,255,255),2)
            cv2.putText(open_cv_image,'%f' % conf,(x1, y1),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),1)
      
         self.get_logger().info('Custom detection almost Done')
         w = x2-x1
         h = y2-y1

         self.from_xy_to_car_pos(w,h) 

         self.img_result = open_cv_image
         self.get_logger().info('Custom detection Done')
         self.finish_process = True
         # cv2.imshow('Processed', )
      else :
         self.get_logger().info('Custom detection exit None')         





def main(args=None):
    rclpy.init(args=args)
    ImgProcess = ImgProcessNode()
    rclpy.spin(ImgProcess)
    ImgProcess.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()