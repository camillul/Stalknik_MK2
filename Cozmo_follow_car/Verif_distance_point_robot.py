import numpy as np

def translation(vector):
    return np.array([[1, 0, 0, vector[0]],
                     [0, 1, 0, vector[1]],
                     [0, 0, 1, vector[2]],
                     [0, 0, 0, 1]])

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

L1 = 8
L2 = 16.3
L3 = 49
L4 = 9.2
alpha = np.deg2rad(15)

T_rc = translation([L4,0,L3]) @ Ry(alpha) @ translation ([L2,0,-L1])

T_rc = T_rc @ ([[0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]])

print(T_rc)

# Matrix of the Cozmo camera (pinhole model) given by "Calibration_Camera.py"
# MatrixCamera = np.array([[293.34562141,   0.,         167.74886443],
#                             [  0.,         292.63154389, 107.3285527 ],
#                             [  0.,           0.,           1.        ]])

MatrixCamera = np.array([[291.41193986,   0.,        170.58829057],
                        [  0.,         291.0430028, 108.7210315 ],
                        [  0.,           0.,           1.        ]])
                            

# fx and fy are the focal lengths expressed in pixel units
fx = MatrixCamera[0][0]
fy = MatrixCamera[1][1]

# (cx,cy) is the image center
cx = MatrixCamera[0][2]
cy = MatrixCamera[1][2]

# x' = x/z
# y' = y/z
# u = fx * x' + cx
# v = fy * y' + cy
# With z=1, then x' = x, y'=y 

#Distance du point par rapport au repère du robot
u=171
# v=163 #10cm
# v=115.5 #15cm
v=93  #20cm


# (u,v) are the coordinates of the projection point in pixels
x = (u - cx)/fx
y = (v - cy)/fy

# print(x,y)

C_r = T_rc @ np.array ([0,0,0,1])
P_r = T_rc @ np.array ([x,y,1,1])

gamma = -C_r[2]/(P_r[2]-C_r[2])

# print(C_r)
# print(P_r)
# print(gamma)

xp = C_r[0] + gamma * (P_r[0] - C_r[0])
yp = C_r[1] + gamma * (P_r[1] - C_r[1])

print(xp,yp)