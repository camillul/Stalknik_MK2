import cozmo
import time
import cv2
import numpy

from PIL import Image, ImageTk
# from simple_image_viewer import SimpleImageViewer


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

    #image_view = SimpleImageViewer(w=320,h=240)

    #out = cv2.VideoWriter('outpy2.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (320,240))

    for i in range(100) :
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
            #out.write(open_cv_image)
            cv2.imshow('frame',open_cv_image) # Image Display
            cv2.imwrite("Calibration {0}.jpg".format(i),open_cv_image)

        
        if cv2.waitKey(33) == 27:
            break

    #out.release()
    
cozmo.run_program(stream_camera)