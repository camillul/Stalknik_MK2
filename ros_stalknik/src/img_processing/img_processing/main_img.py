import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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

        

        # self.declare_parameters(
        #     namespace='Stalknik',
        #     parameters=[
        #         ('IsTest',None)
        #         ('my_video_file',None),
        #     ])


        self.declare_parameter('/Stalknik/IsTest',False)
        self.IsTest = self.get_parameter('/Stalknik/IsTest').get_parameter_value()

        self.declare_parameter('/Stalknik/my_video_file','error')
        self.my_video_file = self.get_parameter('/Stalknik/my_video_file').get_parameter_value().string_value

        
        self.i = 0


    def img_callback(self):

        if self.IsTest:
            self.get_logger().info('TEST Video')
            cap = cv2.VideoCapture(self.my_video_file) 

            while(cap.isOpened()): 
                ret, frame = cap.read() 
                if ret == True: 
                    cv2.imshow('Frame', frame) 
                    msg = Image()
                    msg.header.stamp = Node.get_clock(self).now().to_msg()
                    msg.header.frame_id = 'TestVideo'
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


                    if cv2.waitKey(25) & 0xFF == ord('q'): 
                        break
                
                
                else:  
                    break
                
                cap.release() 
            # self.imagePub.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))
            # self.get_logger().info('Publishing an image')
            # frame = self.cv_image
            # processes image data and converts to ros 2 message


def main(args=None):
    rclpy.init(args=args)
    Img = ImgNode()
    rclpy.spin(Img)
    Img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()