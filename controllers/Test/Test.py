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
import base64
import sys
import os
import tempfile

import ikpy
from ikpy.chain import Chain

try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")
try:
    import cv2
except ImportError:
    sys.exit("Warning: 'cv2' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")


State = Enum('State', ['IDLE', 'WALK', 'FRONT_FALL', 'BACK_FALL'])
IKPY_MAX_ITERATIONS = 4

# Set path to store temporary device images
deviceImagePath = os.getcwd()
try:
    imageFile = open(deviceImagePath + "/image.jpg", 'w')
    imageFile.close()
except IOError:
    deviceImagePath = tempfile.gettempdir()


class Wrestler (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.timeStep = int(self.getBasicTimeStep())
        self.state = State.IDLE
        self.startTime = None
        self.currentMotion = None

        # camera
        # self.camera = self.getDevice("CameraTop")
        # self.camera.enable(4 * self.timeStep)

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

        self.left_leg_chain = Chain.from_urdf_file(
            '../../protos/nao.urdf',
            base_elements=['base_link', 'LHipYawPitch'],
            active_links_mask=[False, True, True,
                               True, True, True, True, False]
        )

        self.right_leg_chain = Chain.from_urdf_file(
            '../../protos/nao.urdf',
            base_elements=['base_link', 'RHipYawPitch'],
            active_links_mask=[False, True, True,
                               True, True, True, True, False]
        )

        # L_leg_motors = []
        # for link in left_leg_chain.links:
        #     if link.name != 'base_link' and link.name != "LLeg_effector_fixedjoint":
        #         motor = self.getDevice(link.name)
        #         #motor.setVelocity(1.0)
        #         position_sensor = motor.getPositionSensor()
        #         position_sensor.enable(self.timeStep)
        #         L_leg_motors.append(motor)

        self.start_joint = [0, 0, 0, -0.523, 1.047, -0.524, 0, 0]

        # arm motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")

        self.RElbowRoll = self.getDevice("RElbowRoll")
        self.LElbowRoll = self.getDevice("LElbowRoll")

        self.RElbowYaw = self.getDevice("RElbowYaw")
        self.LElbowYaw = self.getDevice("LElbowYaw")

        # leg motors
        self.RHipYawPitch = self.getDevice("RHipYawPitch")
        self.LHipYawPitch = self.getDevice("LHipYawPitch")

        self.RHipRoll = self.getDevice("RHipRoll")
        self.LHipRoll = self.getDevice("LHipRoll")

        self.RHipPitch = self.getDevice("RHipPitch")
        self.LHipPitch = self.getDevice("LHipPitch")

        self.RKneePitch = self.getDevice("RKneePitch")
        self.LKneePitch = self.getDevice("LKneePitch")

        self.RAnklePitch = self.getDevice("RAnklePitch")
        self.LAnklePitch = self.getDevice("LAnklePitch")

        self.RAnkleRoll = self.getDevice("RAnkleRoll")
        self.LAnkleRoll = self.getDevice("LAnkleRoll")

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

            # self._image_processing()

            # compute desired feet position from feet trajectory
            x, z = self._compute_desired_position(t)
            right_target_joints = self.left_leg_chain.inverse_kinematics(
                [x[0], 0.05, z[0]],
                [0, 0, 0],
                initial_position=self.start_joint,
                max_iter=IKPY_MAX_ITERATIONS,
                orientation_mode='all'
            )
            self.RHipYawPitch.setPosition(right_target_joints[1])
            self.RHipRoll.setPosition(right_target_joints[2])
            self.RHipPitch.setPosition(right_target_joints[3])
            self.RKneePitch.setPosition(right_target_joints[4])
            self.RAnklePitch.setPosition(right_target_joints[5])
            self.RAnkleRoll.setPosition(right_target_joints[6])

            left_target_joints = self.left_leg_chain.inverse_kinematics(
                [x[1], 0.05, z[1]],
                [0, 0, 0],
                initial_position=self.start_joint,
                max_iter=IKPY_MAX_ITERATIONS,
                orientation_mode='all'
            )
            self.LHipYawPitch.setPosition(left_target_joints[1])
            self.LHipRoll.setPosition(left_target_joints[2])
            self.LHipPitch.setPosition(left_target_joints[3])
            self.LKneePitch.setPosition(left_target_joints[4])
            self.LAnklePitch.setPosition(left_target_joints[5])
            self.LAnkleRoll.setPosition(left_target_joints[6])

            """self._update_accelerometerAverage()
            if self.accelerometerAverage[0] < -7:
                self.state = State.FRONT_FALL
            if self.accelerometerAverage[0] > 7:
                self.state = State.BACK_FALL
            if self.accelerometerAverage[1] < -7:
                self.RShoulderRoll.setPosition(-1.2)
            if self.accelerometerAverage[1] > 7:
                self.LShoulderRoll.setPosition(1.2)
            self.stateAction(t)"""

    def _compute_desired_position(self, t):
        step_period = 0.7
        # More intuitive to make the angle spin clockwise
        theta = -(2 * np.pi * t / step_period) % (2 * np.pi)
        h = -0.3
        step_size = 0.04
        ground_clearance = 0.025
        ground_penetration = 0.005
        x = np.zeros(2)
        x[0] = step_size * np.cos(theta)
        x[1] = -x[0]
        z = np.zeros(2)
        if theta < np.pi:
            z[0] = h + ground_clearance * np.sin(theta)
            z[1] = h - ground_penetration * np.sin(theta)
        else:
            z[0] = h + ground_penetration * np.sin(theta)
            z[1] = h - ground_clearance * np.sin(theta)
        return x, z

    def _update_accelerometerAverage(self):
        # Moving average on self.HISTORY_STEPS steps
        self.accelerometerHistory.pop(0)
        self.accelerometerHistory.append(self.accelerometer.getValues())
        self.accelerometerAverage = [
            sum(x)/self.HISTORY_STEPS for x in zip(*self.accelerometerHistory)]

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
        if self.currentMotion.isOver():
            self.state = State.WALK

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
