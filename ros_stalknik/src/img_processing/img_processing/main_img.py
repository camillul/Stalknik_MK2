import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
# from cv_bridge import CvBridge
import numpy as np
import time
from rclpy.qos import qos_profile_sensor_data



def cv2_to_imgmsg(cv_image, encode ="mono8"):
    """
    This function allow to pass cv_image with ROS messages

    mono8: CV_8UC1, grayscale image

    mono16: CV_16UC1, 16-bit grayscale image

    bgr8: CV_8UC3, color image with blue-green-red color order

    rgb8: CV_8UC3, color image with red-green-blue color order

    bgra8: CV_8UC4, BGR color image with an alpha channel

    rgba8: CV_8UC4, RGB color image with an alpha channel
    """

    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encode
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')


        # Here we declare our parameters from YAML file
        self.declare_parameter('/Stalknik/IsTest',False)
        self.IsTest = self.get_parameter('/Stalknik/IsTest').get_parameter_value()
        self.declare_parameter('/Stalknik/my_video_file',"file:///Users/rikic/Documents/Projet/Stalknik_MK2/PFE_video/outpy2.avi")
        self.my_video_file = self.get_parameter('/Stalknik/my_video_file').get_parameter_value().string_value

        # ****_sub and ****_pub are initialization for publisher and subscriber 
        # args : message type, "topic name", quality of service)
        self.image_pub = self.create_publisher(Image, 'image', qos_profile_sensor_data )

        if self.IsTest:
            self.get_logger().info('TEST Video')
            self.cap = cv2.VideoCapture(self.my_video_file)
            self.img_callback()
        else :
            timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.img_callback)
        
        self.i= 0


    def img_callback(self):
        """
        This function is callback and raise at some event
        It will publish the image that must be processed for car detection

        """

        if self.IsTest:
            while(self.cap.isOpened()): 
                ret, frame = self.cap.read() 
                if ret == True: 
                    cv2.imshow('from_img_acquisition', frame) 

                    self.image_pub.publish(cv2_to_imgmsg(frame))
                    
                    self.get_logger().info('%d Images Published' % self.i)
                    self.i += 1

                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        self.cap.release() 
                        cv2.destroyAllWindows() 
                        pass
                    
                    
                else:  
                    self.cap.release() 
                    cv2.destroyAllWindows()                
                    pass
            else:  
                self.cap.release() 
                cv2.destroyAllWindows()                
                pass
            


def main(args=None):
    rclpy.init(args=args)
    Img = ImgNode()
    rclpy.spin(Img)
    Img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()