import cozmo
import cv2
import numpy

def stream_camera(robot: cozmo.robot.Robot):
    
    #### Robot initialisation ####

    #Lift initialisation
    robot.set_lift_height(100.0).wait_for_completed()

    #Head initialisation
    robot.set_head_angle(cozmo.util.Angle(numpy.deg2rad(-15))).wait_for_completed()

    #Enable camera stream and set color to grayscale (better resolution)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled  = False

    for i in range(150) :
        latest_image = robot.world.latest_image #Save latest image send by the camera

        if latest_image is not None:

            im = latest_image.raw_image
            open_cv_image = numpy.array(im) 

            cv2.imshow('frame',open_cv_image) # Image Display
            cv2.imwrite("Cozmo {0}.jpg".format(i),open_cv_image) # Image Saved
        
        if cv2.waitKey(33) == 27:
            break
    
cozmo.run_program(stream_camera)