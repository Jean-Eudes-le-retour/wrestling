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
import sys
from scipy.spatial.transform import Rotation as R
sys.path.append('..')
from utils.sensors import Accelerometer
from utils.fsm import FiniteStateMachine
from utils.motion import Current_motion_manager, Motion_library
import utils.image

from ikpy.chain import Chain

try:
    import numpy as np
    np.set_printoptions(suppress=True)
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

        # camera
        self.camera = self.getDevice("CameraTop")
        self.camera.enable(self.timeStep)

        # accelerometer
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.timeStep)

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
        self.current_motion = Current_motion_manager()
        self.library = Motion_library()

    def run(self):
        self.leds[0].set(0x0000ff)
        self.leds[1].set(0x0000ff)
        self.current_motion.set(self.library.get('Stand'))
        self.fsm.transition_to('BLOCKING_MOTION')

        while self.step(self.timeStep) != -1:
            self.detectFall()
            self.fsm.execute_action()

    def _compute_desired_position(self, t, x_pos_norm):
        step_period = 0.5
        # More intuitive to make the angle spin clockwise
        theta = -(2 * np.pi * t / step_period) % (2 * np.pi)
        h = -0.29
        step_size = 0.04
        ground_clearance = 0.04
        ground_penetration = 0.005
        x = np.zeros(2)
        step_length_modifier = 0.04
        x[0] =  (step_size - x_pos_norm * step_length_modifier * (x_pos_norm > 0)) * np.cos(theta)
        x[1] = -(step_size + x_pos_norm * step_length_modifier * (x_pos_norm < 0)) * np.cos(theta)
        z = np.zeros(2)
        if theta < np.pi:
            z[0] = h + ground_clearance * np.sin(theta)
            z[1] = h - ground_penetration * np.sin(theta)
        else:
            z[0] = h + ground_penetration * np.sin(theta)
            z[1] = h - ground_clearance * np.sin(theta)
        return x, z, theta

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
        t = self.getTime()
        # compute desired feet position from feet trajectory
        x_pos = self._get_opponent_x()
        # amount of rotation depends on the x position of the opponent
        x_pos_normalized = (x_pos/80 - 1)
        # need to twist ankle + change step length for each leg
        #   c.f. Learning CPG-based Biped Locomotion with a Policy Gradient Method: Application to a Humanoid Robot
        z_angle_twist = 0.34 * x_pos_normalized
        x, z, theta = self._compute_desired_position(t, x_pos_normalized)
        right_target_commands = self.right_leg_chain.inverse_kinematics(
            [x[0], -0.045, z[0]],
            self._z_rotation_matrix(theta, z_angle_twist),
            initial_position=self.right_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(right_target_commands[1:], self.R_leg_motors):
            motor.setPosition(command)
        self.right_previous_joints = right_target_commands

        left_target_commands = self.left_leg_chain.inverse_kinematics(
            [x[1], 0.045, z[1]],
            self._z_rotation_matrix(theta + np.pi, z_angle_twist),
            initial_position=self.left_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(left_target_commands[1:], self.L_leg_motors):
            motor.setPosition(command)
        self.left_previous_joints = left_target_commands
    
    def _z_rotation_matrix(self, theta, rotation_amount):
        return R.from_rotvec((np.sin(theta/2) * rotation_amount + rotation_amount) * np.array([0, 0, 1])).as_matrix()

    def _get_opponent_x(self):
        img = utils.image.get_cv_image_from_camera(self.camera)
        largest_contour, cx, cy = utils.image.locate_opponent(img)
        output = img.copy()
        if largest_contour is not None:
            cv2.drawContours(output, [largest_contour], 0, (255, 255, 0), 1)
        output = cv2.circle(output, (cx, cy), radius=2,
                            color=(0, 0, 255), thickness=-1)
        utils.image.send_image_to_robot_window(self, output)
        return cx

    def frontFall(self): 
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def backFall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')

# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
