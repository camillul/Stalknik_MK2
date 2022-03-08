import numpy as np
from torch import rad2deg

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
alpha = np.deg2rad(25)

T_rc = translation([L4,0,L3]) @ Ry(alpha) @ translation ([L2,0,-L1])

T_rc = T_rc @ ([[0, 0, 1, 0],
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 0, 1]])

T_rc = T_rc @ np.array ([0,0,0,1])

print (T_rc)