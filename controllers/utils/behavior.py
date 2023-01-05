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
import numpy as np

sys.path.append('..')
from utils.sensors import Accelerometer
from utils.fsm import Finite_state_machine
from utils.motion import Current_motion_manager, Motion_library

class Fall_detection:
    def __init__(self, time_step, robot):
        self.time_step = time_step
        self.robot = robot
        # the Finite State Machine (FSM) is a way of representing a robot's behavior as a sequence of states
        self.fsm = Finite_state_machine(
            states=['NO_FALL', 'BLOCKING_MOTION', 'SIDE_FALL', 'FRONT_FALL', 'BACK_FALL'],
            initial_state='NO_FALL',
            actions={
                'NO_FALL': self.wait,
                'BLOCKING_MOTION': self.pending,
                'SIDE_FALL': self.wait,
                'FRONT_FALL': self.front_fall,
                'BACK_FALL': self.back_fall
            }
        )

        # accelerometer
        self.accelerometer = Accelerometer(robot.getDevice('accelerometer'), self.time_step)

        # Shoulder roll motors
        self.RShoulderRoll = robot.getDevice('RShoulderRoll')
        self.LShoulderRoll = robot.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = Current_motion_manager()
        self.library = Motion_library()

    def check(self):
        if self.detect_fall():
            while self.fsm.current_state != 'NO_FALL':
                # block everything and run the recovery motion until the robot is back on its feet
                self.fsm.execute_action()
                self.robot.step(self.time_step)
                self.detect_fall()
    
    def detect_fall(self):
        """Detect a fall and update the FSM state."""
        self.accelerometer.update_average()
        [acc_x, acc_y, _] = self.accelerometer.get_average()
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
        if self.current_motion.is_over():
            self.current_motion.set(self.library.get('Stand'))
            self.fsm.transition_to('NO_FALL')

    def front_fall(self): 
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def back_fall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')
    
    def wait(self):
        pass

class Gait_manager:
    """Simple gait generator, based on an ellipsoid path.
    Derived from chapter 2 in paper:
    G. Endo, J. Morimoto, T. Matsubara, J. Nakanishi, and G. Cheng,
    “Learning CPG-based Biped Locomotion with a Policy Gradient Method: Application to a Humanoid Robot,”
    The International Journal of Robotics Research, vol. 27, no. 2, pp. 213-228,
    Feb. 2008, doi: 10.1177/0278364907084980.
    """
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        self.theta = 0 # angle of the ellipsoid path
        self.imu = robot.getDevice('inertial unit')
        self.imu.enable(self.time_step)
        self.right_foot_sensor = robot.getDevice('RFsr')
        self.right_foot_sensor.enable(self.time_step)
        self.left_foot_sensor = robot.getDevice('LFsr')
        self.left_foot_sensor.enable(self.time_step)

        self.roll_reflex_factor = 5e-4 # h_VSR in paper
        self.force_reflex_factor = 3e-3/(5.305*9.81) # h_ER/(mass*gravity) in paper
        self.robot_height_offset = 0.31 # desired height for the robot's center of mass
        self.lateral_leg_offset = 0.05 # distance between the center of mass and the feet
        self.step_period = 0.4 # time to complete one step
        self.step_length = 0.01 # distance traveled by the feet in one step
        self.step_height = 0.04 # height of the ellipsoid path
        self.step_penetration = 0.005 # depth of the ellipsoid path
        self.calibration_factor = 0.93
    
    def update_theta(self):
        """Update the angle of the ellipsoid path and clip it to [-pi, pi]"""
        self.theta = -(2 * np.pi * self.robot.getTime() / self.step_period) % (2 * np.pi) - np.pi

    def compute_leg_position(self, is_right, desired_radius=1e3, heading_angle=0):
        """Compute the desired positions of a leg for a desired radius (R > 0 is a right turn)."""
        desired_radius *= self.calibration_factor # actual radius is bigger than the desired one, so we "correct" it
        factor = 1 if is_right else -1 # the math is the same for both legs, except for some signs

        amplitude_x = self.step_length * (desired_radius - factor * self.lateral_leg_offset) / desired_radius
        x = factor * amplitude_x * np.cos(self.theta)
        
        # ellipsoid path
        amplitude_z = self.step_penetration if factor * self.theta < 0 else self.step_height
        # vestibulospinal reflex: corrects the robot's roll
        amplitude_z += factor * self.imu.getRollPitchYaw()[0] * self.roll_reflex_factor
        # extensor response: pushes on the leg when it is on the ground
        force_values = self.right_foot_sensor.getValues() if is_right else self.left_foot_sensor.getValues()
        force_magnitude = np.linalg.norm(np.array([force_values[0], force_values[1], force_values[2]]))
        if force_magnitude > 5:
            amplitude_z += self.force_reflex_factor * force_magnitude
        z = factor * amplitude_z * np.sin(self.theta) - self.robot_height_offset
        
        yaw = - x/(desired_radius - factor * self.lateral_leg_offset)
        y = - (1 - np.cos(yaw)) * (desired_radius - factor * self.lateral_leg_offset)
        if heading_angle != 0:
            x, y = self._rotate(x, y, heading_angle)
        y += - factor * self.lateral_leg_offset
        return x, y, z, yaw
    
    def _rotate(self, x, y, angle):
        """Rotate a point by a given angle."""
        return x * np.cos(angle) - y * np.sin(angle), x * np.sin(angle) + y * np.cos(angle)
