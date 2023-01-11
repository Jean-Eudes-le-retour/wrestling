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
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""

from controller import Robot, Motion

import sys
sys.path.append('..')
from utils.sensors import Accelerometer
from utils.routines import Fall_detection
from utils.gait import Gait_manager
# Eve's locate_opponent() is implemented in this module:
import utils.image


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.1

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = self.getDevice("CameraTop")
        self.camera.enable(self.time_step)
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.time_step)
        self.fall_detector = Fall_detection(self.time_step, self)
        self.gait_manager = Gait_manager(self, self.time_step)
        self.punch = Motion('./Punch.motion')
        self.punch.setLoop(True)

    def run(self):
        # self.punch.play()
        while self.step(self.time_step) != -1:
            if self.getTime() > 1:
                self.fall_detector.check()
                # We need to update the internal theta value of the gait manager at every step:
                self.gait_manager.update_theta()
                self.walk()

    def walk(self):
        """Walk towards the opponent like a homing missile."""
        x_pos_normalized = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight (turning radius very high).
        if abs(x_pos_normalized) > 1e-3:
            desired_radius = self.SMALLEST_TURNING_RADIUS / x_pos_normalized
        else:
            desired_radius = 1e3
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=3.14/2)

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = utils.image.get_cv_image_from_camera(self.camera)
        _, _, horizontal_coordinate = utils.image.locate_opponent(img)
        # utils.image.send_image_to_robot_window(self, img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2/img.shape[1] - 1


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
