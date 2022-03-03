import cozmo
import time
import cv2
import numpy
import torch

def stream_camera(robot: cozmo.robot.Robot):

    for i in range (10):
        robot.drive_wheels(i*5,-i*5)
        time.sleep(0.5)
        print(i)
    robot.drive_wheels(0,0)
    time.sleep(1)
    print(0)
    for i in range (10):
        robot.drive_wheels(i*5,-i*5)
        time.sleep(0.5)
        print(i)
    
cozmo.run_program(stream_camera)