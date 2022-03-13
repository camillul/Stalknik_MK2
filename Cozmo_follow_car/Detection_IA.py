import cozmo
import time
import cv2
import numpy as np
import torch


###### Load pretrained AI Model ######

model = torch.hub.load('ultralytics/yolov5', 'custom', path='./trained_carmodel/car_cozmo.pt')

###### Camera Model and Transform Matrix ######

#Transform Matrix to go from Robot landmark to Camera landmark
T_robot_camera = np.array([[ 0.,         -0.25881905,  0.96592583, 22.87403861],
                            [-1.,          0.,          0.,          0.        ],
                            [ 0.,         -0.96592583, -0.25881905, 37.05384295],
                            [ 0.,          0.,          0.,          1.        ]])

# Matrix of the Cozmo camera (pinhole model) given by "Calibration_Camera.py"
MatrixCamera = np.array([[291.41193986,   0.,        170.58829057],
                        [  0.,         291.0430028, 108.7210315 ],
                        [  0.,           0.,           1.        ]])

# fx and fy are the focal lengths expressed in pixel units
fx = MatrixCamera[0][0]
fy = MatrixCamera[1][1]

# (cx,cy) is the image center
cx = MatrixCamera[0][2]
cy = MatrixCamera[1][2]


# Definition of a function capable of remapping data from one scale to another
def mapping(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

###### Cozmo Program ######

def stream_camera(robot: cozmo.robot.Robot):
    
    #### Robot initialisation ####

    #Lift initialisation
    robot.set_lift_height(100.0).wait_for_completed()

    #Head initialisation
    robot.set_head_angle(cozmo.util.Angle(np.deg2rad(-15))).wait_for_completed()

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
    rot_speed = 0
    l_rot_wheel = 0
    r_rot_wheel = 0
    detection = "start"
    signe = False

    backpack_orange_color = cozmo.lights.Light(cozmo.lights.Color(rgb=(255,127,0)))

    while True:
        latest_image = robot.world.latest_image #Save latest image send by the camera

        if latest_image is not None:

            #OPENCV IMAGE SHOW
            im = latest_image.raw_image
            open_cv_image = np.array(im) 
            #ocvim = cv2.cvtColor(open_cv_image, cv2.COLOR_RGB2BGR) #For colored images

            detection = model(open_cv_image, size=320)  # includes NMS

            results_np = detection.pred[0].cpu().detach().numpy()
            
            for result in results_np:
                xa, ya, xb, yb, conf, cls = result
                if conf > 0.3:
                    x1, y1, x2, y2 = [round(f) for f in [xa, ya, xb, yb]]

                cv2.rectangle(open_cv_image,(x1,y1),(x2,y2),(0,255,255),2)
                #cv2.putText(open_cv_image,'%f' % conf,(x1, y1),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),1)
            
                #detection.print()

            # Car bottom middle point in the image in pixels
            Car_BM_point_u = x1+(x2-x1)/2
            Car_BM_point_v = y2


            # (x,y) are the coordinates of the projection point in pixels
            x = (Car_BM_point_u - cx)/fx
            y = (Car_BM_point_v - cy)/fy

            C_r = T_robot_camera @ np.array ([0,0,0,1])
            P_r = T_robot_camera @ np.array ([x,y,1,1])

            gamma = -C_r[2]/(P_r[2]-C_r[2])

            # (x,y) are the coordinates of the Car bottom middle point in the landmark of the robot
            xp = C_r[0] + gamma * (P_r[0] - C_r[0])
            yp = C_r[1] + gamma * (P_r[1] - C_r[1])


            # Width and Height of the detection rectangle
            w = x2-x1
            h = y2-y1

            # # Control law of the linear speed of Cozmo
            # line_speed = 240-h
            # line_speed = mapping(line_speed,85,210,0,25)
            # line_speed = 1.2**line_speed-1

            # # Control law of the linear speed of Cozmo
            # rot_speed = (160-(x1+(w/2)))
            # #rot_speed=0

            # # if result == "start": 
            # #     # set all of Cozmo's backpack lights to orange
            # #     robot.set_all_backpack_lights(backpack_orange_color)
            # # elif result in results_numpy:
            # #     # set all of Cozmo's backpack lights to green
            # #     robot.set_all_backpack_lights(cozmo.lights.green_light)
            # # else:
            # #     # set all of Cozmo's backpack lights to red
            # #     robot.set_all_backpack_lights(cozmo.lights.red_light)
            # #     #line_speed = 10

            # r_rot_wheel = line_speed+K*rot_speed
            # l_rot_wheel = line_speed-K*rot_speed

            # Control law of the linear speed of Cozmo
            line_speed = xp*0.3
            if line_speed >= 120:
                line_speed = 20
                
            #line_speed = mapping(line_speed,20,500,0,25)
            #line_speed = 1.2**line_speed-1
            #line_speed = 0
            rot_speed_old = rot_speed
            # Control law of the linear speed of Cozmo
            #rot_speed = -yp
            rot_speed = (160-(x1+(w/2)))
            #rot_speed=0
            signe = bool(np.sign(rot_speed)+1)
            signe_old = bool(np.sign(rot_speed_old)+1)
            
            if (signe ^ signe_old) == True and rot_speed_old != 0: 
                rot_speed = 0
                print('change')

            r_rot_wheel = line_speed+K*rot_speed
            l_rot_wheel = line_speed-K*rot_speed
            if xp <= 80:
                r_rot_wheel = 0
                l_rot_wheel = 0

            r_rot_wheel = (int)(r_rot_wheel)
            l_rot_wheel = (int)(l_rot_wheel)

            #print(line_speed,rot_speed, xp,yp, r_rot_wheel,l_rot_wheel)
            
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