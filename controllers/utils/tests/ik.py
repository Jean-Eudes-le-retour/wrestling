import numpy as np
from scipy.spatial.transform import Rotation as R

def rot3x3_to_transf4x4(rot3x3):
    transf4x4 = np.zeros((4, 4))
    transf4x4[0:3, 0:3] = rot3x3
    transf4x4[3, 3] = 1
    return transf4x4

ShoulderOffsetY 	= 98.0
ElbowOffsetY		= 15.0
UpperArmLength		= 105.0
ShoulderOffsetZ		= 100.0
LowerArmLength		= 57.75
HandOffsetX			= 55.95
HandOffsetZ			= 12.31
HipOffsetZ			= 85.0
HipOffsetY			= 50.0
ThighLength			= 100.0
TibiaLength			= 102.9
FootHeight			= 45.11
NeckOffsetZ			= 126.5
CameraBottomX		= 48.8
CameraBottomZ		= 23.81
CameraTopX			= 53.9
CameraTopZ			= 67.9

# Left Leg limits
LHipYawPitchHigh	= 0.7408
LHipYawPitchLow		= -1.1453
LHipRollHigh		= 0.7904
LHipRollLow			= -0.3794
LHipPitchHigh		= 0.4840
LHipPitchLow		= -1.7739
LKneePitchHigh		= 2.1125
LKneePitchLow		= -0.0923
LAnklePitchHigh		= 0.9227
LAnklePitchLow		= -1.1895
LAnkleRollHigh		= 0.7690
LAnkleRollLow		= -0.3978
# Left Right limits
RHipYawPitchHigh	= 0.7408
RHipYawPitchLow		= -1.1453
RHipRollHigh		= 0.4147
RHipRollLow			= -0.7383
RHipPitchHigh		= 0.4856
RHipPitchLow		= -1.7723
RKneePitchHigh		= 2.1201
RKneePitchLow		= -0.1030
RAnklePitchHigh		= 0.9320
RAnklePitchLow		= -1.1864
RAnkleRollHigh		= 0.3886
RAnkleRollLow		= -1.1864

# validation test:
# x,y,z = [0, 0.05, -0.3]
# seems like the z position of the origin is not the same in both code
# Rot = np.eye(3)
# thetas = [0., 0., 0., -0.58785418, 1.15704096, -0.56918678, 0., 0.]

def DH(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
        [0, 0, 0, 1]
    ])

def fLeftLeg(thetas):
    base = np.eye(4)
    base[1, 3] = HipOffsetY
    base[2, 3] = -HipOffsetZ

    T1 = DH(0, -3*np.pi/4, 0, thetas[0]-np.pi/2)
    T2 = DH(0, -np.pi/2, 0, thetas[1]+np.pi/4)
    T3 = DH(0, np.pi/2, 0, thetas[2])
    T4 = DH(-ThighLength, 0, 0, thetas[3])
    T5 = DH(-TibiaLength, 0, 0, thetas[4])
    T6 = DH(0, -np.pi/2, 0, thetas[5])
    Rot = rot3x3_to_transf4x4(R.from_euler('zyx', [np.pi, -np.pi/2, 0]).as_matrix())
    Tend1 = np.eye(4)
    Tend1[2, 3] = -FootHeight
    Tendend = base @ T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ Rot @ Tend1

    rotZ = np.arctan2(Tendend[1, 0], Tendend[0, 0])
    rotY = np.arctan2(-Tendend[2, 0], np.sqrt(Tendend[2, 1]**2 + Tendend[2, 2]**2))
    rotX = np.arctan2(Tendend[2, 1], Tendend[2, 2])

    left = np.concatenate((Tendend[0:3, 3], [rotX, rotY, rotZ]))
    return Tendend, left