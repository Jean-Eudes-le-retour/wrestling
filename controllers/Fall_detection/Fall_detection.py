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
Basic controller that uses the Fall_detection class to detect a fall and run a recovery motion.
"""

import sys
from controller import Robot
sys.path.append('..')
from utils.behavior import Fall_detection
from utils.motion import Motion_library, Current_motion_manager

class David (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())
        self.current_motion = Current_motion_manager()
        self.fall_detector = Fall_detection(self.time_step, self)

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        # load motion files
        self.library = Motion_library()

    def run(self):
        self.leds['right'].set(0x0000ff)
        self.leds['left'].set(0x0000ff)
        self.current_motion.set(self.library.get('Stand'))

        while self.step(self.time_step) != -1:
            t = self.getTime()
            if self.current_motion.isOver():
                self.current_motion.set(self.library.get('ForwardLoop'))
            self.fall_detector.check()


# create the Robot instance and run main loop
wrestler = David()
wrestler.run()
