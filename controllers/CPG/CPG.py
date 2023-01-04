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
import matplotlib.pyplot as plt

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

        # CPG structures: amplitude is row 0, and phase is row 1
        self._reset_X()
        self.X_dot = np.zeros((2, 2))
        self._mu = 1
        self._alpha = 0.8
        self._coupling_strength = 1
        self._robot_height = -0.3
        self._step_size = 0.08
        self._ground_clearance = 0.04
        self._ground_penetration = 0.005
        self._omega = 2 * np.pi / 0.6
        self._couple = True
        self.PHI = np.pi * np.flipud(np.diag((1, -1)))
        self.CPG_dot_state = np.expand_dims(self.X_dot.copy(), 2)
        self.CPG_state = np.expand_dims(self.X.copy(), 2)
        self.k_steps = 1

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

        # Force sensors
        self.right_foot_force_sensor = self.getDevice("RFsr")
        self.right_foot_force_sensor.enable(self.timeStep)
        self.right_previous_contact = True
        self.left_foot_force_sensor = self.getDevice("LFsr")
        self.left_foot_force_sensor.enable(self.timeStep)
        self.left_previous_contact = True

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = []
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))

        # getting the motors and kinematic chains from the Nao URDF file
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
                # motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timeStep)
                self.L_leg_motors.append(motor)

        self.R_leg_motors = []
        for link in self.right_leg_chain.links:
            if link.name != 'Base link' and link.name != "RLeg_effector_fixedjoint":
                motor = self.getDevice(link.name)
                # motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timeStep)
                self.R_leg_motors.append(motor)
        
        # Inverse kinematics seed
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
    
    def _reset_X(self):
        # right is 0 and left is 1
        self.X = np.array(
            [[0.1, 0.1],
             [np.pi / 2, 3 * np.pi / 2]]
        )

    def run(self):
        self.leds[0].set(0x0000ff)
        self.leds[1].set(0x0000ff)

        self.stand.play()
        self.currentMotion = self.stand

        fig, axs = plt.subplots(4, 1, sharex=True)
        fig.align_ylabels()

        while self.step(self.timeStep) != -1:
            t = self.getTime()
            self._detect_fall()
            self.stateAction(t)
            if 0: self._plot(axs)

    def _compute_desired_position(self, t):
        # update parameters, integrate
        self._integrate_hopf_equations()

        X = self.X.copy()

        x = np.zeros(2)
        x = -self._step_size * np.cos(X[1, :]) * X[0, :]
        z = np.zeros(2)
        z[X[1, :] < np.pi] = self._robot_height + \
            self._ground_clearance * np.sin(X[1, X[1, :] < np.pi])
        z[X[1, :] >= np.pi] = self._robot_height + \
            self._ground_penetration * np.sin(X[1, X[1, :] >= np.pi])

        return x, z

    def _integrate_hopf_equations(self):
        # bookkeeping - save copies of current CPG states
        X = self.X.copy()
        X_dot = np.zeros((2, 2))
        alpha = self._alpha

        # reset theta if the leg is in contact with the ground
        Lfsv = self.left_foot_force_sensor.getValues()
        left_force = np.linalg.norm(np.array([Lfsv[0], Lfsv[1], Lfsv[2]]))
        is_left_foot_in_contact = left_force > 5
        Rfsv = self.right_foot_force_sensor.getValues()
        right_force = np.linalg.norm(np.array([Rfsv[0], Rfsv[1], Rfsv[2]]))
        is_right_foot_in_contact = right_force > 5
        if is_left_foot_in_contact and self.left_previous_contact == False:
            X[1, 1] = 3 * np.pi / 2
        elif is_right_foot_in_contact and self.right_previous_contact == False:
            X[1, 0] = 3 * np.pi / 2
        self.left_previous_contact = is_left_foot_in_contact
        self.right_previous_contact = is_right_foot_in_contact

        # loop through each leg's oscillator
        for i in range(2):
            # get r_i, theta_i from X
            r, theta = X[0, i], X[1, i]
            # compute r_dot (Equation 6)
            r_dot = alpha * (self._mu - r * r) * r
            theta_dot = self._omega

            # loop through other oscillators to add coupling (Equation 7)
            if self._couple:
                theta_dot += X[0, 1-i] * self._coupling_strength * np.sin(X[1, 1-i] - X[1, i] - self.PHI[i, 1-i])

            # set X_dot[:,i]
            X_dot[:, i] = [r_dot, theta_dot]

        # integrate
        X += X_dot * self.timeStep/1000
        self.X = X
        self.X_dot = X_dot

        # mod phase variables to keep between 0 and 2pi
        self.X[1, :] = self.X[1, :] % (2*np.pi)
        # print(f'left theta: {self.X[1, 1]}')

        self.CPG_dot_state = np.append(self.CPG_dot_state, np.expand_dims(X_dot, 2), axis=2)
        self.CPG_state = np.append(self.CPG_state, np.expand_dims(X, 2), axis=2)
        self.k_steps += 1
    
    def _detect_fall(self):
        self._update_accelerometerAverage()
        if self.accelerometerAverage[0] < -7:
            self.state = State.FRONT_FALL
        if self.accelerometerAverage[0] > 7:
            self.state = State.BACK_FALL
        if self.accelerometerAverage[1] < -7:
            self.RShoulderRoll.setPosition(-1.2)
        if self.accelerometerAverage[1] > 7:
            self.LShoulderRoll.setPosition(1.2)
    
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
        else:
            self._reset_X()
        if self.state == State.FRONT_FALL:
            self.front_fall(t)
        elif self.state == State.BACK_FALL:
            self.back_fall(t)

    def idle(self):
        if self.currentMotion.isOver():
            self.state = State.WALK

    def walk(self, t):
        # compute desired feet position from feet trajectory
        x, z = self._compute_desired_position(t)

        right_target_commands = self.right_leg_chain.inverse_kinematics(
            [x[0], -0.045, z[0]],
            np.eye(3),
            initial_position=self.right_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(right_target_commands[1:], self.R_leg_motors):
            motor.setPosition(command)
        self.right_previous_joints = right_target_commands

        left_target_commands = self.left_leg_chain.inverse_kinematics(
            [x[1], 0.045, z[1]],
            np.eye(3),
            initial_position=self.left_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(left_target_commands[1:], self.L_leg_motors):
            motor.setPosition(command)
        self.left_previous_joints = left_target_commands

    def front_fall(self, time):
        if self.startTime is None:
            self.startTime = time
            self._set_current_motion(self.getUpFront)
        elif self.getUpFront.isOver():
            self.startTime = None
            self.state = State.IDLE
            self._set_current_motion(self.stand)

    def back_fall(self, time):
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
    
    def _plot(self, axs):
        epaisseur = 0.5
        t = np.linspace(0, self.k_steps*self.timeStep/1000, self.k_steps)
        axs[0].plot(t, self.CPG_state[1,1,:], color='tab:blue', linewidth=epaisseur, label='FL phase')
        axs[1].plot(t, self.CPG_dot_state[1,1,:], color='tab:orange', linewidth=epaisseur, label='FL phase dot')
        axs[2].plot(t, self.CPG_state[0,1,:], color='tab:green', linewidth=epaisseur, label='FL amplitude')
        axs[3].plot(t, self.CPG_dot_state[0,1,:], color='tab:red', linewidth=epaisseur, label='FL amplitude dot')
        axs[0].set_ylabel("phase\nLeft [rad]")
        axs[1].set_ylabel("phase\ndot Left [rad/s]")
        axs[2].set_ylabel("amplitude\nLeft [1]")
        axs[3].set_ylabel("amplitude\ndot Left [1/s]")
        axs[3].set_xlabel("time [s]")
        plt.savefig("output.jpg")
        im = plt.imread("output.jpg")
        self._send_image_to_robot_window(im)

    def _send_image_to_robot_window(self, img):
        # fig.canvas.draw()
        # im_bytes = fig.canvas.tostring_rgb()
        # im_arr: image in Numpy one-dim array format.
        _, im_arr = cv2.imencode('.jpg', img)
        im_bytes = im_arr.tobytes()
        im_b64 = base64.b64encode(im_bytes).decode()
        self.wwiSendText("image[camera]:data:image/jpeg;base64," + im_b64)


# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
