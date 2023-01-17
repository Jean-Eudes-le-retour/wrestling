# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ahrs.filters import Mahony, Madgwick, AngularRate
from controller import Robot
from scipy.spatial.transform import Rotation as R
import numpy as np
np.set_printoptions(precision=3, suppress=True)
import sys
sys.path.append('..')
from utils.accelerometer import Accelerometer


class Alice (Robot):
    def __init__(self):
        super().__init__()
        self.time_step_ms = int(self.getBasicTimeStep())
        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.time_step_ms)
        self.accelerometer = Accelerometer(self, self.time_step_ms)
        self.gyroscope = self.getDevice('gyro')
        self.gyroscope.enable(self.time_step_ms)
        self.time_step_s = self.time_step_ms / 1000.
        self.mahony = Mahony(Dt=self.time_step_s, q0=[1., 0., 0., 0.])
        self.madgwick = Madgwick(Dt=self.time_step_s, q0=[1., 0., 0., 0.])
        self.angular_rate = AngularRate(Dt=self.time_step_s, q0=[1., 0., 0., 0.])
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

    def run(self):
        self.RShoulderPitch.setPosition(1.3)
        self.LShoulderPitch.setPosition(1.3)
        Q_mahony = np.array([1., 0., 0., 0.])
        Q_madgwick = np.array([1., 0., 0., 0.])
        Q_angular_rate = np.array([1., 0., 0., 0.])
        Q_manual_angular_rate = np.array([1., 0., 0., 0.])
        while self.step(self.time_step_ms) != -1:
            acc = self.accelerometer.get_new_average()
            acc = np.array(acc)
            acc = self.upright_acceleration(acc)
            gyro = self.gyroscope.getValues()
            gyro = np.array(gyro)
            # R convention [x,y,z,w] R.from_quat(Q_imu).as_euler('xyz') equivalent to self.imu.getRollPitchYaw()
            Q_imu = self.imu.getQuaternion()
            Q_imu = self.from_R_to_ahrs_quaternion_convention(Q_imu)
            Q_mahony = self.mahony.updateIMU(Q_mahony, gyr=gyro, acc=acc)
            Q_madgwick = self.madgwick.updateIMU(Q_madgwick, gyr=gyro, acc=acc)
            Q_angular_rate = self.angular_rate.update(Q_angular_rate, gyr=gyro)
            Q_manual_angular_rate = self.integrate_gyro(Q_manual_angular_rate, gyro)
            print('-' * 20)
            print('IMU :')
            print(self.imu.getRollPitchYaw())
            print(self.quaternion_to_roll_pitch_yaw(Q_imu))
            print('Mahony :')
            print(self.quaternion_to_roll_pitch_yaw(Q_mahony))
            print('Madgwick :')
            print(self.quaternion_to_roll_pitch_yaw(Q_madgwick))
            print('Angular Rate :')
            print(self.quaternion_to_roll_pitch_yaw(Q_angular_rate))
            print('Manual Angular Rate :')
            print(self.quaternion_to_roll_pitch_yaw(Q_manual_angular_rate))
            print('Tilt :')
            print(self.get_tilt(acc))

    def upright_acceleration(self, acc):
        # acc[0] = -acc[0]
        acc[1] = -acc[1]
        acc[2] = -acc[2]
        return acc

    def from_ahrs_quaternion_convention_to_R(self, Q):
        # [w,x,y,z] -> [x,y,z,w]
        Q_prime = np.array([Q[1], Q[2], Q[3], Q[0]])
        return Q_prime

    def from_R_to_ahrs_quaternion_convention(self, Q):
        # [x,y,z,w] -> [w,x,y,z]
        Q_prime = np.array([Q[3], Q[0], Q[1], Q[2]])
        return Q_prime

    def integrate_gyro(self, current_value, gyro_values):
        # input: current_value is a quaternion [w,x,y,z]
        # compute rotation matrix from sample time and gyro rates
        identity_matrix = np.eye(4, 4)
        gyro_matrix = np.array([[0.0, -gyro_values[0], -gyro_values[1], -gyro_values[2]],
                                [gyro_values[0], 0.0, gyro_values[2], -gyro_values[1]],
                                [gyro_values[1], -gyro_values[2], 0.0, gyro_values[0]],
                                [gyro_values[2], gyro_values[1], -gyro_values[0], 0.0]])

        rotation_matrix = self.time_step_s / 2 * gyro_matrix + identity_matrix

        # apply gyro rates to the current quaternion
        new_quaternion = rotation_matrix @ current_value

        # normalize new quaternion
        new_quaternion = new_quaternion / np.linalg.norm(new_quaternion)

        return new_quaternion

    def angle_difference(self, Q1, Q2):
        # Q1 and Q2 are quaternions
        # returns the angle between the two quaternions
        # the angle is in radians
        Q1 = self.from_ahrs_quaternion_convention_to_R(Q1)
        Q2 = self.from_ahrs_quaternion_convention_to_R(Q2)
        R1 = R.from_quat(Q1)
        R2 = R.from_quat(Q2)
        angle = R1.inv() * R2
        angle = angle.as_euler('xyz')
        return angle

    def get_tilt(self, acc):
        # acc is a 3D vector
        # returns the tilt of the robot
        # the tilt is in radians
        ax, ay, az = acc
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        yaw = 0.0
        return np.array([roll, pitch, yaw])

    def quaternion_to_roll_pitch_yaw(self, Q):
        # Q is a quaternion [w,x,y,z]
        # returns the roll pitch and yaw of the robot
        # the angles are in radians
        Q = self.from_ahrs_quaternion_convention_to_R(Q)
        R_orientation = R.from_quat(Q)
        roll, pitch, yaw = R_orientation.as_euler('xyz')
        return np.array([roll, pitch, yaw])


# create the Robot instance and run main loop
wrestler = Alice()
wrestler.run()
