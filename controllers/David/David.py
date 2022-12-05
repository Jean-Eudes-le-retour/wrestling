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
from enum import Enum

State = Enum('State', ['IDLE', 'WALK', 'FRONT_FALL', 'BACK_FALL'])


class Wrestler (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.timeStep = int(self.getBasicTimeStep())
        self.state = State.IDLE
        self.startTime = None
        self.currentMotion = None

        # accelerometer
        self.accelerometer = self.getDevice("accelerometer")
        self.accelerometer.enable(self.timeStep)
        self.accelerometerAverage = [0]*3
        self.HISTORY_STEPS = 10
        self.accelerometerHistory = [[0]*3]*self.HISTORY_STEPS

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = []
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))

        # load motion files
        self.forwards = Motion('../motions/ForwardLoop.motion')
        self.forwards.setLoop(True)
        self.stand = Motion('../motions/Stand.motion')
        self.getUpFront = Motion('../motions/GetUpFront.motion')
        self.getUpBack = Motion('../motions/GetUpBack.motion')

    def run(self):
        self.leds[0].set(0x0000ff)
        self.leds[1].set(0x0000ff)

        self.stand.play()
        self.currentMotion = self.stand

        while self.step(self.timeStep) != -1:
            t = self.getTime()
            # Moving average on self.HISTORY_STEPS steps
            self.accelerometerHistory.pop(0)
            self.accelerometerHistory.append(self.accelerometer.getValues())
            self.accelerometerAverage = [sum(x)/self.HISTORY_STEPS for x in zip(*self.accelerometerHistory)]

            if self.accelerometerAverage[0] < -7:
                self.state = State.FRONT_FALL
            if self.accelerometerAverage[0] > 7:
                self.state = State.BACK_FALL
            self.stateAction(t)

    def stateAction(self, t):
        if self.state == State.IDLE:
            self.idle()
        elif self.state == State.WALK:
            self.walk()
        elif self.state == State.FRONT_FALL:
            self.frontFall(t)
        elif self.state == State.BACK_FALL:
            self.backFall(t)

    def idle(self):
        pass
        # if self.currentMotion.isOver():
        #     self.state = State.WALK

    def walk(self):
        if self.currentMotion != self.forwards:
            self._set_current_motion(self.forwards)

    def frontFall(self, time):
        if self.startTime is None:
            self.startTime = time
            self._set_current_motion(self.getUpFront)
        elif self.getUpFront.isOver():
            self.startTime = None
            self.state = State.IDLE
            self._set_current_motion(self.stand)

    def backFall(self, time):
        if self.startTime is None:
            self.startTime = time
            self._set_current_motion(self.getUpBack)
        elif self.getUpBack.isOver():
            self.startTime = None
            self.state = State.IDLE
            self._set_current_motion(self.stand)

    def _set_current_motion(self, motion):
        self.currentMotion.stop()
        self._reset_isOver_flag(self.currentMotion)
        self.currentMotion = motion
        motion.play()

    def _reset_isOver_flag(self, motion):
        motion.play()
        motion.stop()


# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
