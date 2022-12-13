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
from scipy.spatial.transform import Rotation as R

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


class Wrestler (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        # IK solver is quite heavy, so we will run only every 2 time steps
        self.timeStep = int(2 * self.getBasicTimeStep())
        self.state = State.IDLE
        self.startTime = None
        self.currentMotion = None

        # camera
        self.camera = self.getDevice("CameraTop")
        self.camera.enable(4 * self.timeStep)

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

        self.L_leg_motors = []
        for link in self.left_leg_chain.links:
            if link.name != 'Base link' and link.name != "LLeg_effector_fixedjoint":
                motor = self.getDevice(link.name)
                #motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timeStep)
                self.L_leg_motors.append(motor)
        
        self.R_leg_motors = []
        for link in self.right_leg_chain.links:
            if link.name != 'Base link' and link.name != "RLeg_effector_fixedjoint":
                motor = self.getDevice(link.name)
                #motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timeStep)
                self.R_leg_motors.append(motor)

        self.left_previous_joints = [0, 0, 0, -0.523, 1.047, -0.524, 0, 0]
        self.right_previous_joints = [0, 0, 0, -0.523, 1.047, -0.524, 0, 0]

        # arm motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")

        self.RElbowRoll = self.getDevice("RElbowRoll")
        self.LElbowRoll = self.getDevice("LElbowRoll")

        self.RElbowYaw = self.getDevice("RElbowYaw")
        self.LElbowYaw = self.getDevice("LElbowYaw")

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

            self._update_accelerometerAverage()
            if self.accelerometerAverage[0] < -7:
                self.state = State.FRONT_FALL
            if self.accelerometerAverage[0] > 7:
                self.state = State.BACK_FALL
            if self.accelerometerAverage[1] < -7:
                self.RShoulderRoll.setPosition(-1.2)
            if self.accelerometerAverage[1] > 7:
                self.LShoulderRoll.setPosition(1.2)
            self.stateAction(t)

    def _compute_desired_position(self, t):
        step_period = 0.5
        # More intuitive to make the angle spin clockwise
        theta = -(2 * np.pi * t / step_period) % (2 * np.pi)
        h = -0.29
        step_size = 0.04
        ground_clearance = 0.04
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
        return x, z, theta

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
            self.walk(t)
        elif self.state == State.FRONT_FALL:
            self.frontFall(t)
        elif self.state == State.BACK_FALL:
            self.backFall(t)

    def idle(self):
        if self.currentMotion.isOver():
            self.state = State.WALK

    def walk(self, t):
        # compute desired feet position from feet trajectory
        x, z, theta = self._compute_desired_position(t)
        x_pos = self._image_processing()
        # amount of rotation depends on the x position of the opponent
        virage = 0.12 * (x_pos/80 - 1)
        right_target_commands = self.right_leg_chain.inverse_kinematics(
            [x[0], -0.045, z[0]],
            self._z_rotation_matrix(theta, virage),
            initial_position=self.right_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(right_target_commands[1:], self.R_leg_motors):
            motor.setPosition(command)
        self.right_previous_joints = right_target_commands

        left_target_commands = self.left_leg_chain.inverse_kinematics(
            [x[1], 0.045, z[1]],
            self._z_rotation_matrix(theta + np.pi, virage),
            initial_position=self.left_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(left_target_commands[1:], self.L_leg_motors):
            motor.setPosition(command)
        self.left_previous_joints = left_target_commands
    
    def _z_rotation_matrix(self, theta, rotation_amount):
        return R.from_rotvec((np.sin(theta/2) * rotation_amount + rotation_amount) * np.array([0, 0, 1])).as_matrix()

    def _image_processing(self):
        img = self._get_cv_image_from_camera()

        # The robot is supposed to be located at a concentration of high color changes (big Laplacian values)
        laplacian = cv2.Laplacian(img, cv2.CV_8U, ksize=3)
        # those spikes are then smoothed out using a Gaussian blur to get blurry blobs
        blur = cv2.GaussianBlur(laplacian, (0, 0), 2)
        # We apply a threshold to get a binary image of potential robot locations
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
        # the binary image is then dilated to merge small groups of blobs together
        closing = cv2.morphologyEx(
            thresh, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)))
        # the largest contour is then picked and its centroid is used as the robot's location
        contours, hierarchy = cv2.findContours(
            closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # prepares robot window image
        output = img.copy()

        # get centroid of the largest contour, default to the image's center
        # and draw the contour + centroid on the output image
        if len(contours) > 0:
            largest_contour = contours[0]
            M = cv2.moments(largest_contour)
            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            except ZeroDivisionError:
                cx = img.shape[1]//2
                cy = img.shape[0]//2

            cv2.drawContours(
                output, [largest_contour], 0, (255, 255, 0), 1)
        else:
            # get center of image
            cx = img.shape[1]//2
            cy = img.shape[0]//2
        output = cv2.circle(output, (cx, cy), radius=2,
                            color=(0, 0, 255), thickness=-1)

        self._send_image_to_robot_window(output)
        return cx

    def _send_image_to_robot_window(self, img):
        # im_arr: image in Numpy one-dim array format.
        _, im_arr = cv2.imencode('.jpg', img)
        im_bytes = im_arr.tobytes()
        im_b64 = base64.b64encode(im_bytes).decode()
        self.wwiSendText("image[camera]:data:image/jpeg;base64," + im_b64)

    def _get_cv_image_from_camera(self):
        return np.frombuffer(self.camera.getImage(), np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))

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
