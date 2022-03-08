import cozmo
import time
import cv2
import numpy

from PIL import Image, ImageTk
# from simple_image_viewer import SimpleImageViewer

cascade_src = 'cars.xml'

car_cascade = cv2.CascadeClassifier(cascade_src)

#Proportional corrector coefficients
K=0.2
C=0.3

def stream_camera(robot: cozmo.robot.Robot):
    
    #### Robot initialisation ####

    #Lift initialisation
    robot.set_lift_height(100.0).wait_for_completed()

    #Head initialisation
    robot.set_head_angle(cozmo.util.Angle(numpy.deg2rad(0))).wait_for_completed()

    #Enable camera stream and set color to grayscale (better resolution)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled  = False

    #image_view = SimpleImageViewer(w=320,h=240)

    while True:
        latest_image = robot.world.latest_image #Save latest image send by the camera

        if latest_image is not None:
            # PIL IMAGE SHOW
            # im = latest_image.raw_image
            # open_cv_image = numpy.array(im) 
            # ocvim = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR)
            # image = ImageTk.PhotoImage(im)
            # image_view.update_image(image)

            #OPENCV IMAGE SHOW
            im = latest_image.raw_image
            open_cv_image = numpy.array(im) 
            #ocvim = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR) #For colored images

            cars = car_cascade.detectMultiScale(open_cv_image, 1.1, 2)

            for (x,y,w,h) in cars:
                cv2.rectangle(open_cv_image,(x,y),(x+w,y+h),(0,255,255),2)

                line_speed = 240-h

                if line_speed < 100:
                    line_speed = 0

                rot_speed = (160-(x+(w/2)))

                r_rot_wheel = C*line_speed+K*rot_speed
                l_rot_wheel = C*line_speed-K*rot_speed
                print(line_speed)
                #print(r_rot_wheel,l_rot_wheel)
                robot.drive_wheels(l_rot_wheel, r_rot_wheel)

            if cars == ():
                robot.drive_wheels(0, 0)
                robot.set_all_backpack_lights(cozmo.lights.red_light)
            else : 
                robot.set_all_backpack_lights(cozmo.lights.green_light)
                
                
            #time.sleep(0.1)
            cv2.imshow('frame',open_cv_image) # Image Display
            #cv2.imwrite("template {0}.jpg".format(i),open_cv_image)

        #
        if cv2.waitKey(33) == 27:
            break

cozmo.run_program(stream_camera)