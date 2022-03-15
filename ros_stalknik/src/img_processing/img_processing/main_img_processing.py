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


      # Here we declare our parameters from YAML file
      self.declare_parameter('/Stalknik/my_yolo_file','error')
      self.my_yolo_file = self.get_parameter('/Stalknik/my_yolo_file').get_parameter_value().string_value
      self.declare_parameter('/Stalknik/my_trained_model','error')
      self.my_trained_model = self.get_parameter('/Stalknik/my_trained_model').get_parameter_value().string_value
      self.declare_parameter('/Stalknik/IsCustom',True)
      self.IsCustom = self.get_parameter('/Stalknik/IsCustom').get_parameter_value()

        # ****_sub and ****_pub are initialization for publisher and subscriber 
        # args : message type, "topic name", quality of service)
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
      

      self.car_pos_vec =  np.array([0,0,0],np.float)
      self.drone_pos_vec = np.array([0,0,0],np.float)

      self.T_robot_camera = np.array([[ 0.,         -0.25881905,  0.96592583, 22.87403861],
                                 [-1.,          0.,          0.,          0.        ],
                                 [ 0.,         -0.96592583, -0.25881905, 37.05384295],
                                 [ 0.,          0.,          0.,          1.        ]])

      # Matrix of the Cozmo camera (pinhole model) given by "Calibration_Camera.py"
      self.MatrixCamera = np.array([[291.41193986,   0.,        170.58829057],
                              [  0.,         291.0430028, 108.7210315 ],
                              [  0.,           0.,           1.        ]])

      # fx and fy are the focal lengths expressed in pixel units
      self.fx = self.MatrixCamera[0][0]
      self.fy = self.MatrixCamera[1][1]

      # (cx,cy) is the image center
      self.cx = self.MatrixCamera[0][2]
      self.cy = self.MatrixCamera[1][2]



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
      
      timer_period = 2  # seconds
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

   def update_car_pos(self,xc,yc):
      """
      update car position with directly xc,yc which are position along respectively x, y robot axis reference
      """

      self.car_pos_vec[0] = (float(xc) *float(0.0010))  #float(0.0010)allow to change from mm to m
      self.car_pos_vec[1] = (float(yc) *float(0.0010))
      self.car_pos_vec[2] = 0

   def img_process_callback(self):
      """
        This function is callback and raise at some event
        It will publish the car position from image processing

      """
      self.get_logger().info('img_callback')
      self.finish_process = False
      if self.IsCustom :
         self.get_logger().info('Custom processing')
         self.custom_car_detection(self.new_image_to_process)


      else :
         self.car_detection(self.new_image_to_process)

      # self.finish_process ensure that the image processing have been done
      if (self.finish_process == True) : 


         car_pose_msg = Pose()
         car_pose_msg.position.x = float(self.car_pos_vec[0])
         car_pose_msg.position.y = float(self.car_pos_vec[1])
         car_pose_msg.position.z = float(self.car_pos_vec[2])

         # TODO:Ricky:04/03/2022: Implement rotation detection

         self.car_position_pub.publish(car_pose_msg)
         self.get_logger().info('Publishing : "%s"' % car_pose_msg.position)

         cv2.imshow('img_processing', self.img_result)

         
         cv2.waitKey(33) == 27
            
         







   def img_callback(self,msg):


      try:
         self.new_image_to_process = self.imgmsg_to_cv2(msg)
         # self.new_image_to_process = np.array(self.frame, dtype=np.uint8)
         self.get_logger().info("Image Received")
         if self.new_image_to_process is not None:
            cv2.imshow('Received by img_processing', self.new_image_to_process)
            cv2.waitKey(1)
      except: 
         pass


      

   def car_detection(self,img):
      detector = ObjectDetection()
      detector.setModelTypeAsYOLOv3()
      detector.setModelPath(self.my_yolo_file)
      detector.loadModel()
      returned_image, detections, extracted_objects = detector.detectObjectsFromImage(input_image=img,input_type="array",output_type="array",extract_detected_objects=True, minimum_percentage_probability=20)
      
      self.finish_process = True
      
   def custom_car_detection(self,img):

      x1 = 0
      y1 = 0
      x2 = 320
      y2 = 240
      signe = False

      self.get_logger().info('Custom detection if')
      if self.new_image_to_process is not None:
         self.get_logger().info('Custom detection begin')
         #OPENCV IMAGE SHOW
         im = self.new_image_to_process
         open_cv_image = im
         # Detection by inference to find the car in the image with pretrained AI Model loaded before
         detection = self.model(open_cv_image, size=320)

         # Result of the Detection
         results_np = detection.pred[0].cpu().detach().numpy()
         
         # Draw a rectangle arround the car if the detection ratio is more than 30%
         # It uses xa, ya, xb, yb which are the coordinates of the 4 corners of the box detection
         # given by the result variable of the detection "result_np"
         for result in results_np:
               xa, ya, xb, yb, conf, cls = result
               if conf > 0.3:
                  x1, y1, x2, y2 = [round(f) for f in [xa, ya, xb, yb]]

               cv2.rectangle(open_cv_image,(x1,y1),(x2,y2),(0,255,255),2)

         # Car bottom middle point in the image in pixels
         Car_BM_point_u = x1+(x2-x1)/2
         Car_BM_point_v = y2

         # (x,y) are the coordinates of the projection point in pixels
         x = (Car_BM_point_u - self.cx)/self.fx
         y = (Car_BM_point_v - self.cy)/self.fy

         # Calculation of the coordinates of the point of intersection between the plane of the ground 
         # and the vector corresponding to the selected point of the image, along the optical axis
         C_r = self.T_robot_camera @ np.array ([0,0,0,1])
         P_r = self.T_robot_camera @ np.array ([x,y,1,1])

         gamma = -C_r[2]/(P_r[2]-C_r[2])

         # (xp,yp) are the coordinates of the Car bottom middle point in the robot landmark
         xp = C_r[0] + gamma * (P_r[0] - C_r[0])
         yp = C_r[1] + gamma * (P_r[1] - C_r[1])

         self.update_car_pos(xp,yp)
         # Width and Height of the detection rectangle
         # w = x2-x1
         # h = y2-y1
         # self.from_xy_to_car_pos(w,h) 

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