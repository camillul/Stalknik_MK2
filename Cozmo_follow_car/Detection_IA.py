import cozmo
import time
import cv2
import numpy
import torch

from PIL import Image, ImageTk


#Load pretrained AI Model 
model = torch.hub.load('ultralytics/yolov5', 'custom', path='./trained_carmodel/car_cozmo.pt')

#Proportional corrector coefficients
K=0.2
C=0.3

def stream_camera(robot: cozmo.robot.Robot):
    
    #### Robot initialisation ####

    #Lift initialisation
    robot.move_lift(5)
    time.sleep(0.5)
    robot.move_lift(0)

    #Head initialisation
    robot.move_head(-2)
    time.sleep(0.3)
    robot.move_head(0.7)
    time.sleep(0.3)
    robot.move_head(0)

    #Enable camera stream and set color to grayscale (better resolution)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled  = False

    while True:
        latest_image = robot.world.latest_image #Save latest image send by the camera

        if latest_image is not None:

            #OPENCV IMAGE SHOW
            im = latest_image.raw_image
            open_cv_image = numpy.array(im) 
            #ocvim = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR) #For colored images

            detection = model(open_cv_image, size=320)  # includes NMS

            results_numpy = detection.pred[0].cpu().detach().numpy()
            
            for result in results_numpy:
                x1, y1, x2, y2, conf, cls = result
                x1, y1, x2, y2 = [round(f) for f in [x1, y1, x2, y2]]

                cv2.rectangle(open_cv_image,(x1,y1),(x2,y2),(0,255,255),2)
                #cv2.putText(open_cv_image,'%f' % conf,(x1, y1),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),1)
            
            #detection.print()
            w = x2-x1
            h = y2-y1

            print(results_numpy)
            line_speed = 240-h

            if x1 < 100:
                line_speed = 0

            rot_speed = (160-(x1+(w/2)))

            r_rot_wheel = C*line_speed+K*rot_speed
            l_rot_wheel = C*line_speed-K*rot_speed
            print(line_speed)
            #print(r_rot_wheel,l_rot_wheel)
            robot.drive_wheels(l_rot_wheel, r_rot_wheel)

            # if cars == ():
            #     robot.drive_wheels(0, 0)
            #     robot.set_all_backpack_lights(cozmo.lights.red_light)
            # else : 
            #     robot.set_all_backpack_lights(cozmo.lights.green_light)
                
                
            #time.sleep(0.1)
            cv2.imshow('frame',open_cv_image) # Image Display
            #cv2.imwrite("template {0}.jpg".format(i),open_cv_image)

        #
        if cv2.waitKey(33) == 27:
            break

cozmo.run_program(stream_camera)