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
Demonstrates how to use the Motion_library class to play a motion file.
Beats Alice by moving forwards and therefore having a higher coverage.
"""

import sys
from controller import Robot
sys.path.append('..') # adding the utils folder to get access to some custom helper functions, have a look at it
from utils.motion import Motion_library

class Bob (Robot):
    def __init__(self):
        super().__init__()
        # to load all the motions from the motion folder, we use the Motion_library class:
        self.library = Motion_library()
        
        # we initialize the shoulder pitch motors using the Robot.getDevice() function:
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")
    def run(self):
        # to play a motion from the library, we use the play() function as follows:
        self.library.play('Forwards50')
        
        # to control a motor, we use the setPosition() function:
        self.RShoulderPitch.setPosition(1.3)
        self.LShoulderPitch.setPosition(1.3)
        # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
        # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

        time_step = int(self.getBasicTimeStep())
        while self.step(time_step) != -1:
            pass


# create the Robot instance and run main loop
wrestler = Bob()
wrestler.run()
