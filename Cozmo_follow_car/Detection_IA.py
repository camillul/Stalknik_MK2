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

    #Proportional corrector coefficients of rotational speed
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

    while True:
        #Save latest image send by the camera
        latest_image = robot.world.latest_image 

        if latest_image is not None:
            
            # Conversion of the image in numpy array, so that it can be used with OpenCV
            im = latest_image.raw_image
            open_cv_image = np.array(im) 

            # Detection by inference to find the car in the image with pretrained AI Model loaded before
            detection = model(open_cv_image, size=320)

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
            x = (Car_BM_point_u - cx)/fx
            y = (Car_BM_point_v - cy)/fy

            # Calculation of the coordinates of the point of intersection between the plane of the ground 
            # and the vector corresponding to the selected point of the image, along the optical axis
            C_r = T_robot_camera @ np.array ([0,0,0,1])
            P_r = T_robot_camera @ np.array ([x,y,1,1])

            gamma = -C_r[2]/(P_r[2]-C_r[2])

            # (xp,yp) are the coordinates of the Car bottom middle point in the robot landmark
            xp = C_r[0] + gamma * (P_r[0] - C_r[0])
            yp = C_r[1] + gamma * (P_r[1] - C_r[1])

            # Width and Height of the detection rectangle
            w = x2-x1
            h = y2-y1

            # Control law of the linear speed of Cozmo
            line_speed = xp*0.3
            if line_speed >= 120:
                line_speed = 20
                
            # Control law of the rotationnal speed of Cozmo
            rot_speed_old = rot_speed
            rot_speed = (160-(x1+(w/2)))

            # Dection of a change of rotation direction
            # Cozmo robot has priorization for a null command send its motors so, we want the
            # to send him 
            # TO BE TESTED
            signe = bool(np.sign(rot_speed)+1)
            signe_old = bool(np.sign(rot_speed_old)+1)
            
            if (signe ^ signe_old) == True and rot_speed_old != 0: 
                rot_speed = 0
                print('change')

            # Combination of rotational and linear speed to send a command to Cozmo wheels.
            r_rot_wheel = line_speed+K*rot_speed
            l_rot_wheel = line_speed-K*rot_speed

            # If the distance between the robot center (in the robot landmark) and the bottom of the rear
            # of the car is less than 80mm, stop the robot
            if xp <= 80:
                r_rot_wheel = 0
                l_rot_wheel = 0

            # Cast command to int, because float takes more time to be processed by the robot, and then,
            # the lantency between the command send and the effectivness of it is reduced
            r_rot_wheel = (int)(r_rot_wheel)
            l_rot_wheel = (int)(l_rot_wheel)
            
            # Send the wheels command to Cozmo
            robot.drive_wheels(l_rot_wheel, r_rot_wheel)
            
            # Image Display
            cv2.imshow('landmark',open_cv_image) 

        # Press "esc" to quit the program
        if cv2.waitKey(33) == 27:
            break

# Robot does not drive off charger at the stard of program
cozmo.robot.Robot.drive_off_charger_on_connect = False

# Starting main program on Cozmo
cozmo.run_program(stream_camera)