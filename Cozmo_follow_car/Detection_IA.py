import cozmo
import time
import cv2
import numpy
import torch


#Load pretrained AI Model 
model = torch.hub.load('ultralytics/yolov5', 'custom', path='./trained_carmodel/car_cozmo.pt')

def mapping(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def stream_camera(robot: cozmo.robot.Robot):
    
    #### Robot initialisation ####

    #Lift initialisation
    robot.set_lift_height(100.0).wait_for_completed()

    #Head initialisation
    robot.set_head_angle(cozmo.util.Angle(numpy.deg2rad(0))).wait_for_completed()

    #Enable camera stream and set color to grayscale (better resolution)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled  = False

    #Proportional corrector coefficients rotation
    K=0.2

    # Variables initialisation
    x1 = 0
    y1 = 0
    x2 = 320
    y2 = 240
    l_rot_wheel = 0
    r_rot_wheel = 0
    detection = "start"

    backpack_orange_color = cozmo.lights.Light(cozmo.lights.Color(rgb=(255,127,0)))

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
                xa, ya, xb, yb, conf, cls = result
                if conf > 0.3:
                    x1, y1, x2, y2 = [round(f) for f in [xa, ya, xb, yb]]

                cv2.rectangle(open_cv_image,(x1,y1),(x2,y2),(0,255,255),2)
                cv2.putText(open_cv_image,'%f' % conf,(x1, y1),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),1)
            
                #detection.print()

            w = x2-x1
            h = y2-y1

            # Control law of the linear speed of Cozmo
            line_speed = 240-h
            line_speed = mapping(line_speed,85,210,0,25)
            line_speed = 1.2**line_speed-1

            # Control law of the linear speed of Cozmo
            rot_speed = (160-(x1+(w/2)))
            #rot_speed=0

            # if result == "start": 
            #     # set all of Cozmo's backpack lights to orange
            #     robot.set_all_backpack_lights(backpack_orange_color)
            # elif result in results_numpy:
            #     # set all of Cozmo's backpack lights to green
            #     robot.set_all_backpack_lights(cozmo.lights.green_light)
            # else:
            #     # set all of Cozmo's backpack lights to red
            #     robot.set_all_backpack_lights(cozmo.lights.red_light)
            #     #line_speed = 10

            r_rot_wheel = line_speed+K*rot_speed
            l_rot_wheel = line_speed-K*rot_speed

            r_rot_wheel = (int)(r_rot_wheel)
            l_rot_wheel = (int)(l_rot_wheel)
            #print(line_speed)
            
            robot.drive_wheels(l_rot_wheel, r_rot_wheel)
                
            #time.sleep(0.1)
            cv2.imshow('frame',open_cv_image) # Image Display
            #cv2.imwrite("template {0}.jpg".format(i),open_cv_image)

        # Press "esc" to quit the program
        if cv2.waitKey(33) == 27:
            break

# Robot does not drive off charger at the stard of program
cozmo.robot.Robot.drive_off_charger_on_connect = False

# Starting main program on Cozmo
cozmo.run_program(stream_camera)