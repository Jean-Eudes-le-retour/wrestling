import math
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
CameraBotomX		= 48.8
CameraBotomZ		= 23.81
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

# RLEG
# End and Base effectors
RotRLeg = R.from_rotvec(np.array([math.pi, -math.pi / 2, 0.0])).as_matrix()
RotRLeg = rot3x3_to_transf4x4(RotRLeg)

t1 = np.array([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, -FootHeight],
               [0, 0, 0, 1]])

RotRLeg = t1.dot(RotRLeg)


# LLEG
# End and Base effectors
TBaseLLeg = np.array([[1, 0, 0, 0],
                      [0, 1, 0, HipOffsetY],
                      [0, 0, 1, -HipOffsetZ],
                      [0, 0, 0, 1]])

TEndLLeg = R.from_rotvec(np.array([math.pi, -math.pi / 2, 0.0])).as_matrix()
TEndLLeg = rot3x3_to_transf4x4(TEndLLeg)
TEndLLeg = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, FootHeight],
                     [0, 0, 0, 1]]).dot(TEndLLeg)

TBaseLLegInv = np.array([[1, 0, 0, 0],
                         [0, 1, 0, -HipOffsetY],
                         [0, 0, 1, HipOffsetZ],
                         [0, 0, 0, 1]])
TBaseLLegInv = np.linalg.inv(TBaseLLegInv)

RotFixLLeg = R.from_rotvec(np.array([math.pi / 2 / 2, 0.0, 0.0])).as_matrix()
RotFixLLeg = rot3x3_to_transf4x4(RotFixLLeg)

t1 = np.array([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, -FootHeight],
               [0, 0, 0, 1]])

TEndLLegInv = R.from_rotvec(np.array([math.pi, -math.pi / 2, 0.0])).as_matrix()
TEndLLegInv = rot3x3_to_transf4x4(TEndLLegInv)
TEndLLegInv = t1.dot(TEndLLegInv)
TEndLLegInv = np.linalg.inv(TEndLLegInv)

def makeTransformation(px, py, pz, rx, ry, rz):
    Transf = np.zeros((4, 4))
    Transf[0, 0] = math.cos(rz) * math.cos(ry)
    Transf[0, 1] = math.cos(rz) * math.sin(ry) * math.sin(rx) - math.sin(rz) * math.cos(rx)
    Transf[0, 2] = math.cos(rz) * math.sin(ry) * math.cos(rx) + math.sin(rz) * math.sin(rx)
    Transf[0, 3] = px
    Transf[1, 0] = math.sin(rz) * math.cos(ry)
    Transf[1, 1] = math.sin(rz) * math.sin(ry) * math.sin(rx) + math.cos(rz) * math.cos(rx)
    Transf[1, 2] = math.sin(rz) * math.sin(ry) * math.cos(rx) - math.cos(rz) * math.sin(rx)
    Transf[1, 3] = py
    Transf[2, 0] = -math.sin(ry)
    Transf[2, 1] = math.cos(ry) * math.sin(rx)
    Transf[2, 2] = math.cos(ry) * math.cos(rx)
    Transf[2, 3] = pz
    Transf[3, 3] = 1
    return Transf

def inverse_left_leg(target_point):
    return_result = []
    TtempTheta5, T4i, T5i, T6i, Ttemp, Ttemp2 = None, None, None, None, None, None

    T = target_point
    Tinit = T

    # Move the start point to the hipyawpitch point
    base = TBaseLLegInv
    base = TBaseLLegInv.dot(T)
    # Move the end point to the anklePitch joint
    base = base.dot(TEndLLegInv)

    # Rotate hipyawpitch joint
    Rot = RotFixLLeg
    Rot = Rot.dot(base)

    # Invert the table, because we need the chain from the ankle to the hip
    Tstart = Rot
    try:
        Rot = np.linalg.inv(Rot)
    except np.linalg.LinAlgError:
        return return_result

    T = Rot

    # Build the rotation table
    side1 = ThighLength
    side2 = TibiaLength

    # Calculate Theta 4
    distancesqrd = np.linalg.norm(T[:3, 3])**2
    theta4 = math.pi - math.acos((side1**2 + side2**2 - distancesqrd) / (2 * side1 * side2)) # ERROR HERE

    if math.isnan(theta4):
        return return_result

    theta6 = math.atan(T[1, 3] / T[2, 3])

    if theta6 < LAnkleRollLow or theta6 > LAnkleRollHigh:
        return return_result

    T6i = R.from_rotvec(np.array([0.0, -math.pi / 2, 0.0, theta6])).as_matrix()
    T6i = T6i.dot(RotRLeg)

    try:
        T6i = np.linalg.inv(T6i)
        Tstart = Tstart.dot(T6i)
        TtempTheta5 = Tstart
        TtempTheta5 = np.linalg.inv(TtempTheta5)
    except np.linalg.LinAlgError:
        return return_result

    for itter in range(2):
        theta4 = (itter == 0) and theta4 or -theta4

        if theta4 < RKneePitchLow or theta4 > RKneePitchHigh:
            continue

        T4i = R.from_rotvec(np.array([-ThighLength, 0.0, 0.0, theta4])).as_matrix()
        up = TtempTheta5[1, 3] * (TibiaLength + ThighLength * math.cos(theta4)) + ThighLength * TtempTheta5[0, 3] * math.sin(theta4)
        down = ThighLength**2 * math.sin(theta4)**2 + (TibiaLength + ThighLength * math.cos(theta4))**2
        theta5 = math.asin(-up / down)
        posOrNegPIt5 = (theta5 >= 0) and math.pi or -math.pi

        if math.isnan(theta5) and up / down < 0:
            theta5 = -math.pi / 2
        elif math.isnan(theta5):
            theta5 = math.pi / 2
        else:
            theta5 = math.pi / 2

        for i in range(2):
            if i == 0 and (theta5 < LAnklePitchLow or theta5 > LAnklePitchHigh):
                continue
            elif i == 1 and (-theta5 < LAnklePitchLow or -theta5 > LAnklePitchHigh):
                continue


            T5i = R.from_rotvec(np.array([0.0, 0.0, posOrNegPIt5, theta5])).as_matrix()
            Ttemp = T5i.dot(T4i)
            Ttemp = Ttemp.dot(Tstart)
            Ttemp2 = Ttemp.dot(TBaseLLeg)
            Ttemp2 = Ttemp2.dot(TEndLLeg)
            Ttemp2 = Ttemp2.dot(Tinit)

            hip_yaw_pitch = math.atan2(Ttemp2[1, 0], Ttemp2[0, 0])
            hip_roll = math.atan2(-Ttemp2[2, 0], Ttemp2[2, 2])
            hip_pitch = math.atan2(Ttemp2[2, 1], Ttemp2[2, 2])
            knee_pitch = theta4
            ankle_pitch = theta5
            ankle_roll = theta6

            return_result.append([hip_yaw_pitch, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll])

    return return_result
