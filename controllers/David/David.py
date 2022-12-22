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

import sys
from controller import Robot, Motion
sys.path.append('../utils')
from accelerometer import Accelerometer
from fsm import FiniteStateMachine
from motion import Current_motion_manager, Motion_library

class David (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.timeStep = int(self.getBasicTimeStep())
        self.fsm = FiniteStateMachine(
            states=['DEFAULT', 'BLOCKING_MOTION', 'FRONT_FALL', 'BACK_FALL'],
            initial_state='DEFAULT'
        )

        # accelerometer
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.timeStep)

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = []
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))

        # Shoulder roll motors
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = Current_motion_manager()

        self.library = Motion_library()

    def run(self):
        self.leds[0].set(0x0000ff)
        self.leds[1].set(0x0000ff)
        self.current_motion.set(self.library.motions['Stand'])
        self.fsm.transition_to('BLOCKING_MOTION')

        while self.step(self.timeStep) != -1:
            t = self.getTime()
            self.detectFall()
            self.stateAction(t)
    
    def detectFall(self):
        ''' Rudimentary fall detector. Manages transitions to the correct state'''
        self.accelerometer.update()
        accelerometer_average = self.accelerometer.getAverage()

        if accelerometer_average[0] < -7:
            self.fsm.transition_to('FRONT_FALL')
        if accelerometer_average[0] > 7:
            self.fsm.transition_to('BACK_FALL')
        if accelerometer_average[1] < -7:
            # Fell on its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
        if accelerometer_average[1] > 7:
            # Fell on its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)

    def stateAction(self, t):
        state = self.fsm.current_state
        if state == 'BLOCKING_MOTION':
            self.pending()
        elif state == 'DEFAULT':
            self.walk()
        elif state == 'FRONT_FALL':
            self.frontFall()
        elif state == 'BACK_FALL':
            self.backFall()

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.isOver():
            self.current_motion.set(self.library.motions['Stand'])
            self.fsm.transition_to('DEFAULT')

    def walk(self):
        if self.current_motion.get() != self.library.motions['ForwardLoop']:
            self.current_motion.set(self.library.motions['ForwardLoop'])

    def frontFall(self): 
        self.fsm.transition_to('BLOCKING_MOTION')
        self.current_motion.set(self.library.motions['GetUpFront'])

    def backFall(self):
        self.current_motion.set(self.library.motions['GetUpBack'])
        self.fsm.transition_to('BLOCKING_MOTION')


# create the Robot instance and run main loop
wrestler = David()
wrestler.run()
