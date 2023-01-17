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

"""
Minimalist controller example for the Robot Wrestling Tournament.
Demonstrates how to play a simple motion file.
"""

from controller import Robot
import sys
sys.path.append('..')
from utils.pose_estimator import PoseEstimator
import numpy as np
np.set_printoptions(precision=3, suppress=True)


class Alice (Robot):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.time_step)
        # algorithm list: tilt, mahony, madgwick, angular_rate, manual_angular_rate
        self.pose_estimator = PoseEstimator(self, self.time_step, algorithm='manual_angular_rate')
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

    def run(self):
        self.RShoulderPitch.setPosition(0)
        self.LShoulderPitch.setPosition(0)
        while self.step(self.time_step) != -1:  # Mandatory function to make the simulation run
            print(self.imu.getRollPitchYaw())
            print(self.pose_estimator.get_roll_pitch_yaw())


# create the Robot instance and run main loop
wrestler = Alice()
wrestler.run()
