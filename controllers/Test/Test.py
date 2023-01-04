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
from controller import Robot
sys.path.append('..')
from utils.sensors import Accelerometer
from utils.fsm import FiniteStateMachine
from utils.motion import Current_motion_manager, Motion_library

class David (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.timeStep = int(self.getBasicTimeStep())
        # the Finite State Machine (FSM) is a way of representing a robot's behavior as a sequence of states
        self.fsm = FiniteStateMachine(
            states=['DEFAULT', 'BLOCKING_MOTION', 'FRONT_FALL', 'BACK_FALL'],
            initial_state='DEFAULT',
            actions={
                'BLOCKING_MOTION': self.pending,
                'DEFAULT': self.walk,
                'FRONT_FALL': self.frontFall,
                'BACK_FALL': self.backFall
            }
        )

        # accelerometer
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.timeStep)

        # inertial unit to read the robot's orientation
        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timeStep)

        # GPS to read the robot's position
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        # Shoulder roll motors
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = Current_motion_manager()
        self.library = Motion_library()

    def run(self):
        self.leds['right'].set(0x0000ff)
        self.leds['left'].set(0x0000ff)
        self.current_motion.set(self.library.get('Stand'))
        self.fsm.transition_to('BLOCKING_MOTION')

        while self.step(self.timeStep) != -1:
            t = self.getTime()
            self.detectFall()
            # print(self.inertialUnit.getRollPitchYaw())
            self.fsm.execute_action()
    
    def detectFall(self):
        """Detect a fall and update the FSM state."""
        self.accelerometer.update()
        [acc_x, acc_y, _] = self.accelerometer.getAverage()
        if acc_x < -7:
            self.fsm.transition_to('FRONT_FALL')
        elif acc_x > 7:
            self.fsm.transition_to('BACK_FALL')
        if acc_y < -7:
            # Fell to its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
        elif acc_y > 7:
            # Fell to its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.isOver():
            self.current_motion.set(self.library.get('Stand'))
            self.fsm.transition_to('DEFAULT')

    def walk(self):
        if self.current_motion.get() != self.library.get('ForwardLoop'):
            self.current_motion.set(self.library.get('ForwardLoop'))

    def frontFall(self): 
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def backFall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')


# create the Robot instance and run main loop
wrestler = David()
wrestler.run()
