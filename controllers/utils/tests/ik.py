"""
Port of the inverse kinematics code from this paper:
N. Kofinas, “Forward and inverse kinematics for the NAO humanoid robot,” Diploma Thesis,
Technical University of Crete, Greece, 2012,
available at: https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf
C++ code available at: https://github.com/kouretes/NAOKinematics
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

# Constants from C++ code
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
# thetas = [LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll]
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

def orientation_to_transform(orientation):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('ZYX', orientation).as_matrix()
    return T

def orientation_from_transform(T):
    roll = np.arctan2(T[2, 1], T[2, 2])
    pitch = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
    yaw = np.arctan2(T[1, 0], T[0, 0])
    return np.array([roll, pitch, yaw])

def left_leg_transform_matrixes(thetas):
    A_base_0 = np.eye(4)
    A_base_0[1, 3] = HipOffsetY
    A_base_0[2, 3] = -HipOffsetZ
    T_0_1 = DH(0, -3*np.pi/4, 0, thetas[0]-np.pi/2)
    T_1_2 = DH(0, -np.pi/2, 0, thetas[1]+np.pi/4)
    T_2_3 = DH(0, np.pi/2, 0, thetas[2])
    T_3_4 = DH(-ThighLength, 0, 0, thetas[3])
    T_4_5 = DH(-TibiaLength, 0, 0, thetas[4])
    T_5_6 = DH(0, -np.pi/2, 0, thetas[5])
    Rot_zy = orientation_to_transform([np.pi, -np.pi/2, 0])
    A_6_end = np.eye(4)
    A_6_end[2, 3] = -FootHeight
    return A_base_0, T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6, Rot_zy, A_6_end

def right_leg_transform_matrixes(thetas):
    A_base_0 = np.eye(4)
    A_base_0[1, 3] = -HipOffsetY
    A_base_0[2, 3] = -HipOffsetZ
    T_0_1 = DH(0, -np.pi/4, 0, thetas[0]-np.pi/2)
    T_1_2 = DH(0, -np.pi/2, 0, thetas[1]-np.pi/4)
    T_2_3 = DH(0, np.pi/2, 0, thetas[2])
    T_3_4 = DH(-ThighLength, 0, 0, thetas[3])
    T_4_5 = DH(-TibiaLength, 0, 0, thetas[4])
    T_5_6 = DH(0, -np.pi/2, 0, thetas[5])
    Rot_zy = orientation_to_transform([np.pi, -np.pi/2, 0])
    A_6_end = np.eye(4)
    A_6_end[2, 3] = -FootHeight
    return A_base_0, T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6, Rot_zy, A_6_end

def forward_left_leg(thetas):
    A_base_0, T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6, Rot_zy, A_6_end = left_leg_transform_matrixes(thetas)
    T_base_end = A_base_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6 @ Rot_zy @ A_6_end
    return np.concatenate((T_base_end[0:3, 3], orientation_from_transform(T_base_end)))

def transform_from_position_and_orientation(position, orientation):
    T = orientation_to_transform(orientation)
    T[0:3, 3] = position
    return T

def inverse_left_leg(x, y, z, roll, pitch, yaw):
    def get_A_base_0():
        A_base_0 = np.eye(4)
        A_base_0[1, 3] = HipOffsetY
        A_base_0[2, 3] = -HipOffsetZ
        return A_base_0
    
    def get_T_3_4(theta_4):
        T_3_4 = DH(-ThighLength, 0, 0, theta_4)
        return T_3_4
    
    def get_T_4_5(theta_5):
        T_4_5 = DH(-TibiaLength, 0, 0, theta_5)
        return T_4_5
    
    def get_T_5_6(theta_6):
        T_5_6 = DH(0, -np.pi/2, 0, theta_6)
        return T_5_6
    
    def get_Rot_zy():
        Rot_zy = orientation_to_transform([np.pi, -np.pi/2, 0])
        return Rot_zy
    
    def get_A_6_end():
        A_6_end = np.eye(4)
        A_6_end[2, 3] = -FootHeight
        return A_6_end
    
    T = transform_from_position_and_orientation([x, y, z], [yaw, pitch, roll])
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = [0, 0, 0, 0, 0, 0]
    A_base_0 = get_A_base_0()
    A_6_end = get_A_6_end()
    T_hat = np.linalg.inv(A_base_0) @ T @ np.linalg.inv(A_6_end)
    T_tilde = orientation_to_transform([0, 0, np.pi/4]) @ T_hat
    T_prime = np.linalg.inv(T_tilde)
    px_prime, py_prime, pz_prime = T_prime[0:3, 3]
    d = np.linalg.norm([px_prime, py_prime, pz_prime])
    theta_4_double_prime = np.pi - np.arccos((ThighLength**2 + TibiaLength**2 - d**2)/(2*ThighLength*TibiaLength))
    theta_4 = []
    for theta_4_test in [theta_4_double_prime, -theta_4_double_prime]:
        if LKneePitchLow < theta_4_test < LKneePitchHigh:
            theta_4.append(theta_4_test)
    theta_6 = np.arctan(py_prime/pz_prime)
    T_tilde_prime = T_tilde @ np.linalg.inv(get_T_5_6(theta_6) @ get_Rot_zy())
    T_double_prime = np.linalg.inv(T_tilde_prime)
    theta_5 = []
    for theta_4_test in theta_4:
        numerator = T_double_prime[1, 3] * (TibiaLength + ThighLength * np.cos(theta_4_test)) + ThighLength * T_double_prime[0, 3] * np.sin(theta_4_test)
        denominator = ThighLength**2 * np.sin(theta_4_test)**2 + (TibiaLength + ThighLength * np.cos(theta_4_test))**2
        theta_5_prime = np.arcsin(- numerator / denominator)
        for theta_5_test in [theta_5_prime, (np.pi if theta_5_prime >= 0 else - np.pi) - theta_5_prime]:
            print('theta_5_test', theta_5_test)
            if LAnklePitchLow < theta_5_test < LAnklePitchHigh:
                theta_5.append(theta_5_test)
    for theta_5_test in theta_5:
        for theta_4_test in theta_4:
            T_3_4 = get_T_3_4(theta_4_test)
            T_4_5 = get_T_4_5(theta_5_test)
            T_triple_prime = T_tilde_prime @ np.linalg.inv(T_3_4 @ T_4_5)
            theta_2 = []
            theta_2_prime = np.arccos(T_triple_prime[1, 2])
            for theta_2_test in [theta_2_prime - np.pi/4, -theta_2_prime - np.pi/4]:
                if LHipRollLow < theta_2_test < LHipRollHigh:
                    theta_2.append(theta_2_test)
                else:
                    continue
                theta_3_prime = np.arcsin(T_triple_prime[1, 1] / np.sin(theta_2_test + np.pi/4))
                print('T_triple_prime[1, 1]', T_triple_prime[1, 1])
                theta_3 = []
                for theta_3_test in [theta_3_prime, (np.pi if theta_3_prime >= 0 else - np.pi) - theta_3_prime]:
                    print('theta_3_test', theta_3_test)
                    if LHipPitchLow < theta_3_test < LHipPitchHigh:
                        theta_3.append(theta_3_test)
                if len(theta_3) == 0:
                    continue
                theta_1_prime = np.arccos(T_triple_prime[0, 2] / np.sin(theta_2_test + np.pi/4))
                theta_1 = []
                for theta_1_test in [theta_1_prime + np.pi/2, -theta_1_prime + np.pi/2]:
                    print
                    if LHipYawPitchLow < theta_1_test < LHipYawPitchHigh:
                        theta_1.append(theta_1_test)
    return np.array([0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, 0], dtype=object)