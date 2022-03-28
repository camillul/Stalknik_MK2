import numpy as np

# Definition of the Translation Matrix
def translation(vector):
    return np.array([[1, 0, 0, vector[0]],
                     [0, 1, 0, vector[1]],
                     [0, 0, 1, vector[2]],
                     [0, 0, 0, 1]])

# Definition of the 3 Rotations Matrix
def Rx(alpha):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(alpha), -np.sin(alpha), 0],
                     [0, np.sin(alpha), np.cos(alpha), 0],
                     [0, 0, 0, 1]])

def Ry(alpha):
    return np.array([[np.cos(alpha), 0, np.sin(alpha), 0],
                     [0, 1, 0, 0],
                     [-np.sin(alpha), 0, np.cos(alpha), 0],
                     [0, 0, 0, 1]])
        
def Rz(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0, 0],
                     [np.sin(alpha), np.cos(alpha), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

# Dimensions of the Cozmo robot
L1 = 8
L2 = 16.3
L3 = 49
L4 = 9.2
alpha = np.deg2rad(15)

# Transformation matrix from the robot landmark to the camera landmark.
T_rc = translation([L4,0,L3]) @ Ry(alpha) @ translation ([L2,0,-L1])

T_rc = T_rc @ ([[0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]])

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

# Distance of the point from the robot landmark
u=170
v=163 #point at 10cm
# v=115.5 #point at 15cm
#v=93  #point at 20cm

# (x,y) are the coordinates of the projection point in pixels
x = (u - cx)/fx
y = (v - cy)/fy

# Calculation of the coordinates of the point of intersection between the plane of the ground 
# and the vector corresponding to the selected point of the image, along the optical axis
C_r = T_rc @ np.array ([0,0,0,1])
P_r = T_rc @ np.array ([x,y,1,1])

gamma = -C_r[2]/(P_r[2]-C_r[2])

# Coordinates of the point in the robot landmark.
xp = C_r[0] + gamma * (P_r[0] - C_r[0])
yp = C_r[1] + gamma * (P_r[1] - C_r[1])

print(xp,yp)