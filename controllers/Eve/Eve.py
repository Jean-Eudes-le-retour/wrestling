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
Controller example for the Robot Wrestling Tournament.
David beats Charlie by getting up after being knocked down.
Demonstrates how to use a sensor, here an accelerometer to detect a fall.
Depending on the fall direction, the robot will play a different motion, which is implemented by a simple Finite State Machine.
"""

from controller import Robot, Motion
import sys
sys.path.append('..')
from utils.behavior import Fall_detection
import utils.image
from utils.fsm import FiniteStateMachine
from utils.motion import Current_motion_manager
from utils.utils import Average

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


class Eve (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.timeStep = int(self.getBasicTimeStep())

        self.fsm = FiniteStateMachine(
            states=['CHOOSE_ACTION', 'BLOCKING_MOTION'],
            initial_state='CHOOSE_ACTION',
            actions={
                'CHOOSE_ACTION': self.choose_action,
                'BLOCKING_MOTION': self.pending
            }
        )

        # camera
        self.camera = self.getDevice("CameraTop")
        self.camera.enable(self.timeStep)

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = []
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))

        # arm motors for getting up from a side fall
        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")

        self.fall_detector = Fall_detection(self.timeStep, self)

        self.current_motion = Current_motion_manager()
        # load motion files
        self.motions = {
            'SideStepLeft': Motion('../motions/SideStepRightLoop.motion'),
            'TurnLeft':     Motion('../motions/TurnLeft20.motion'),
            'TurnRight':    Motion('../motions/TurnRight20.motion'),
        }

        self.opponent_position = Average(dimensions=1)

    def run(self):
        self.leds[0].set(0x0000ff)
        self.leds[1].set(0x0000ff)

        while self.step(self.timeStep) != -1:
            self.opponent_position.updateAverage(self._get_normalized_opponent_horizontal_position())
            self.fall_detector.check()
            self.fsm.execute_action()

    def choose_action(self):
        if self.opponent_position.average < -0.4:
            self.current_motion.set(self.motions['TurnLeft'])
        elif self.opponent_position.average > 0.4:
            self.current_motion.set(self.motions['TurnRight'])
        else:
            self.current_motion.set(self.motions['SideStepLeft'])
        self.fsm.transition_to('BLOCKING_MOTION')

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.isOver():
            self.fsm.transition_to('CHOOSE_ACTION')

    def _get_normalized_opponent_horizontal_position(self):
        """Returns the horizontal position of the opponent in the image, normalized to [-1, 1]
            and sends an annotated image to the robot window."""
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
wrestler = Eve()
wrestler.run()
