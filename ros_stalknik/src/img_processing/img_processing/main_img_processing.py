import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
import numpy as np

import time
import cv2
import numpy
import torch
torch.cuda.is_available()
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

      self.carPositionPub = self.create_publisher(Pose, 'car_position', 10)
      timer_period = 0.5  # seconds
      # self.timer = self.create_timer(timer_period, self.img_process_callback)
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
      self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.my_trained_model,force_reload=True)



   def from_xy_to_car_pos(self,x,y):
      # this function give x,y position of the car for a drone, this one must be above 
      

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

      while True:
        

         if self.latest_image is not None:

            #OPENCV IMAGE SHOW
            im = self.latest_image.raw_image
            open_cv_image = numpy.array(im) 

            detection = model(open_cv_image, size=320)  # includes NMS

            results_numpy = detection.pred[0].cpu().detach().numpy()
            
            for result in results_numpy:
               xa, ya, xb, yb, conf, cls = result
               if conf > 0.3:
                  x1, y1, x2, y2 = [round(f) for f in [xa, ya, xb, yb]]

               cv2.rectangle(open_cv_image,(x1,y1),(x2,y2),(0,255,255),2)
               cv2.putText(open_cv_image,'%f' % conf,(x1, y1),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),1)
         

            w = x2-x1
            h = y2-y1

            self.from_xy_to_car_pos(w,h)
   
            msg = Pose()
            msg.position.x = self.car_pos[0]
            msg.position.y = self.car_pos[1]
            msg.position.z = self.car_pos[2]

            # TODO:Ricky:04/03/2022: Implement rotation detection



            self.carPositionPub.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.position)


         if cv2.waitKey(33) == 27:
            break






   def img_callback(self,msg):

      height = msg.height
      width = msg.width
      self.resolution_x = width
      self.resolution_y = height
      channel = msg.step//msg.width
      frame = np.reshape(msg.data, (height, width, channel))

      self.latest_image = frame
      # from frame, we can start post-processing
      self.get_logger().info("Image Received")


   def car_detection(self,img):
      detector = ObjectDetection()
      detector.setModelTypeAsYOLOv3()
      detector.setModelPath(self.my_yolo_file)
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