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

import sys
sys.path.append('..')
from utils.sensors import Accelerometer
from utils.fsm import FiniteStateMachine
from utils.motion import Current_motion_manager, Motion_library

class Fall_detection:
    def __init__(self, timeStep, robot):
        self.timeStep = timeStep
        self.robot = robot
        # the Finite State Machine (FSM) is a way of representing a robot's behavior as a sequence of states
        self.fsm = FiniteStateMachine(
            states=['NO_FALL', 'BLOCKING_MOTION', 'SIDE_FALL', 'FRONT_FALL', 'BACK_FALL'],
            initial_state='NO_FALL',
            actions={
                'NO_FALL': self.wait,
                'BLOCKING_MOTION': self.pending,
                'SIDE_FALL': self.wait,
                'FRONT_FALL': self.frontFall,
                'BACK_FALL': self.backFall
            }
        )

        # accelerometer
        self.accelerometer = Accelerometer(robot.getDevice('accelerometer'), self.timeStep)

        # Shoulder roll motors
        self.RShoulderRoll = robot.getDevice('RShoulderRoll')
        self.LShoulderRoll = robot.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = Current_motion_manager()
        self.library = Motion_library()

    def check(self):
        if self.detectFall():
            print('Fall detected - running recovery motion')
            while self.fsm.current_state != 'NO_FALL':
                # block everything and run the recovery motion until the robot is back on its feet
                self.detectFall()
                self.fsm.execute_action()
                self.robot.step(self.timeStep)
    
    def detectFall(self):
        """Detect a fall and update the FSM state."""
        self.accelerometer.update()
        [acc_x, acc_y, _] = self.accelerometer.getAverage()
        fall = False
        if acc_x < -7:
            self.fsm.transition_to('FRONT_FALL')
            fall = True
        elif acc_x > 7:
            self.fsm.transition_to('BACK_FALL')
            fall = True
        if acc_y < -7:
            # Fell to its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
            self.fsm.transition_to('SIDE_FALL')
            fall = True
        elif acc_y > 7:
            # Fell to its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)
            self.fsm.transition_to('SIDE_FALL')
            fall = True
        return fall

    def pending(self):
        # waits for the current motion to finish
        if self.current_motion.isOver():
            self.current_motion.set(self.library.get('Stand'))
            self.fsm.transition_to('NO_FALL')

    def frontFall(self): 
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def backFall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')
    
    def wait(self):
        pass