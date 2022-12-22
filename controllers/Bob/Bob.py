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

"""Minimalist controller example for the Robot Wrestling Tournament.
   Beats Alice by moving forwards and therefore having a higher coverage."""

import sys
from controller import Robot
sys.path.append('../utils') # adding the utils folder to get access to some helper functions
from motion import Motion_library

class Bob (Robot):
    def __init__(self):
        super().__init__()
        # to load all the motions from the motion folder, we use the Motion_library class:
        self.library = Motion_library()
        
        # initializing shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")
    def run(self):
        # to play a motion from the library, we use the play() function as follows:
        self.library.play('Forwards50')
        
        # to control a motor, we use the setPosition() function:
        self.RShoulderPitch.setPosition(1.57)  # arms in front, zombie mode
        self.LShoulderPitch.setPosition(1.57)
        # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor

        timeStep = int(self.getBasicTimeStep())  # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        while self.step(timeStep) != -1:
            pass


# create the Robot instance and run main loop
wrestler = Bob()
wrestler.run()
