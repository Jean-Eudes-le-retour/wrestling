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

from controller import Robot, Motion
import sys
from scipy.spatial.transform import Rotation as R
sys.path.append('..')
from utils.routines import Fall_detection
from utils.motion import Current_motion_manager
from utils.sensors import Accelerometer
from utils.utils import Kinematics, Gait_manager
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


class Fatima (Robot):
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        # IK is heavy, so we only compute it every 2 time steps
        self.kinematics = Kinematics(self, 4 * self.time_step)

        self.camera = self.getDevice("CameraTop")
        self.camera.enable(self.time_step)
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.time_step)
        self.fall_detector = Fall_detection(self.time_step, self)
        self.gait_manager = Gait_manager(self, self.time_step)
        self.current_motion = Current_motion_manager()
        self.current_motion.set(Motion('../motions/Stand.motion'))

    def run(self):
        while self.step(self.time_step) != -1:
            if self.current_motion.is_over():
                self.fall_detector.check()
                self.gait_manager.update_theta()
                self.walk()

    def walk(self):
        x_pos_normalized = self._get_normalized_opponent_x()
        desired_radius = 0.1 / x_pos_normalized if abs(x_pos_normalized) > 1e-3 else 1e3
        desired_radius = 1
        heading_angle = np.pi/2
        x, y, z, yaw = self.gait_manager.compute_leg_position(is_right=True, desired_radius=desired_radius, heading_angle=heading_angle)
        right_target_commands = self.kinematics.ik_right_leg(
            [x, y, z],
            R.from_rotvec(yaw * np.array([0, 0, 1])).as_matrix()
        )
        for command, motor in zip(right_target_commands[1:], self.kinematics.R_leg_motors):
            motor.setPosition(command)

        x, y, z, yaw = self.gait_manager.compute_leg_position(is_right=False, desired_radius=desired_radius, heading_angle=heading_angle)
        left_target_commands = self.kinematics.ik_left_leg(
            [x, y, z],
            R.from_rotvec(yaw * np.array([0, 0, 1])).as_matrix()
        )
        for command, motor in zip(left_target_commands[1:], self.kinematics.L_leg_motors):
            motor.setPosition(command)

    def _get_normalized_opponent_x(self):
        img = utils.image.get_cv_image_from_camera(self.camera)
        largest_contour, vertical, horizontal = utils.image.locate_opponent(img)
        output = img.copy()
        if largest_contour is not None:
            cv2.drawContours(output, [largest_contour], 0, (255, 255, 0), 1)
            output = cv2.circle(output, (horizontal, vertical), radius=2,
                                color=(0, 0, 255), thickness=-1)
        # utils.image.send_image_to_robot_window(self, output)
        if horizontal is None:
            return 0
        return horizontal * 2/img.shape[1] - 1

# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
