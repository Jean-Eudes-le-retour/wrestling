# Copyright 1996-2022 Cyberbotics Ltd.
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
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""

from controller import Robot
from enum import Enum
import sys
from scipy.spatial.transform import Rotation as R
sys.path.append('..')
from utils.behavior import Fall_detection
from utils.sensors import Accelerometer
from utils.utils import Kinematics
import utils.image

try:
    import numpy as np
    np.set_printoptions(suppress=True)
except ImportError:
    sys.exit("Warning: 'numpy' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")
try:
    import cv2
except ImportError:
    sys.exit("Warning: 'cv2' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")


State = Enum('State', ['IDLE', 'WALK', 'FRONT_FALL', 'BACK_FALL'])
IKPY_MAX_ITERATIONS = 4


class Fatima (Robot):
    def __init__(self):
        Robot.__init__(self)
        # IK is heavy, so we only compute it every 2 time steps
        self.time_step = int(self.getBasicTimeStep())
        self.kinematics = Kinematics(self, 2 * self.time_step)

        self.camera = self.getDevice("CameraTop")
        self.camera.enable(self.time_step)
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.time_step)
        self.fall_detector = Fall_detection(self.time_step, self)

    def run(self):
        while self.step(self.time_step) != -1:
            # self.fall_detector.check()
            self.walk()

    def _compute_desired_position(self, t, x_pos_norm):
        step_period = 0.5
        # More intuitive to make the angle spin clockwise
        theta = -(2 * np.pi * t / step_period) % (2 * np.pi)
        h = -0.29
        step_size = 0.04
        ground_clearance = 0.04
        ground_penetration = 0
        x = np.zeros(2)
        step_length_modifier = 0.04
        x[0] =  (step_size - x_pos_norm * step_length_modifier * (x_pos_norm > 0)) * np.cos(theta)
        x[1] = -(step_size + x_pos_norm * step_length_modifier * (x_pos_norm < 0)) * np.cos(theta)
        z = np.zeros(2)
        if theta < np.pi:
            z[0] = h + ground_clearance * np.sin(theta)
            z[1] = h - ground_penetration * np.sin(theta)
        else:
            z[0] = h + ground_penetration * np.sin(theta)
            z[1] = h - ground_clearance * np.sin(theta)
        return x, z, theta

    def walk(self):
        t = self.getTime()
        x_pos_normalized = self._get_normalized_opponent_x()
        # need to twist ankle + change step length for each leg
        #   c.f. Learning CPG-based Biped Locomotion with a Policy Gradient Method: Application to a Humanoid Robot
        z_angle_twist = 0.34 * x_pos_normalized
        x, z, theta = self._compute_desired_position(t, x_pos_normalized)
        right_target_commands = self.kinematics.ik_right_leg([x[0], -0.045, z[0]], self._z_rotation_matrix(theta, z_angle_twist))
        for command, motor in zip(right_target_commands[1:], self.kinematics.R_leg_motors):
            motor.setPosition(command)

        left_target_commands = self.kinematics.ik_left_leg([x[1], 0.045, z[1]], self._z_rotation_matrix(theta, -z_angle_twist))
        for command, motor in zip(left_target_commands[1:], self.kinematics.L_leg_motors):
            motor.setPosition(command)
    
    def _z_rotation_matrix(self, theta, rotation_amount):
        return R.from_rotvec((np.sin(theta/2) * rotation_amount + rotation_amount) * np.array([0, 0, 1])).as_matrix()

    def _get_normalized_opponent_x(self):
        img = utils.image.get_cv_image_from_camera(self.camera)
        largest_contour, vertical, horizontal = utils.image.locate_opponent(img)
        output = img.copy()
        if largest_contour is not None:
            cv2.drawContours(output, [largest_contour], 0, (255, 255, 0), 1)
            output = cv2.circle(output, (horizontal, vertical), radius=2,
                                color=(0, 0, 255), thickness=-1)
        utils.image.send_image_to_robot_window(self, output)
        if horizontal is None:
            return 0
        return horizontal * 2/img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
