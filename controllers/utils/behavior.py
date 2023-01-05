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
    """Simple gait generator, based on an ellipsoid path."""
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        self.theta = 0
        self.robot_height_offset = 0.32
        self.lateral_leg_offset = 0.05
        self.step_period = 0.4
        self.step_length = 0.04
        self.step_height = 0.04
        self.step_penetration = 0.005
    
    def update_theta(self):
        """Update the angle of the ellipsoid path and clip it to [0, 2pi]."""
        self.theta = -(2 * np.pi * self.robot.getTime() / self.step_period) % (2 * np.pi)
    
    def compute_right_leg_position(self, desired_radius):
        """Compute the desired position of the right leg from a desired radius (R > 0 is a right turn).
        Derived from chapter 2 in paper:
        G. Endo, J. Morimoto, T. Matsubara, J. Nakanishi, and G. Cheng,
        “Learning CPG-based Biped Locomotion with a Policy Gradient Method: Application to a Humanoid Robot,”
        The International Journal of Robotics Research, vol. 27, no. 2, pp. 213-228,
        Feb. 2008, doi: 10.1177/0278364907084980.
        """
        amplitude = self.step_length * (desired_radius - self.lateral_leg_offset) / desired_radius
        x = amplitude * np.cos(self.theta)
        if self.theta < np.pi:
            z = self.step_height * np.sin(self.theta) - self.robot_height_offset
        else:
            z = self.step_penetration * np.sin(self.theta) - self.robot_height_offset
        yaw = - x/(desired_radius - self.lateral_leg_offset)
        y = - self.lateral_leg_offset - (1 - np.cos(yaw)) * (desired_radius - self.lateral_leg_offset)
        return x, y, z, yaw
    
    def compute_left_leg_position(self, desired_radius):
        """Compute the desired position of the left leg from a desired radius (R > 0 is a right turn)."""
        amplitude = self.step_length * (desired_radius + self.lateral_leg_offset) / desired_radius
        # print('Left factor', (desired_radius + self.lateral_leg_offset) / desired_radius)
        x = - amplitude * np.cos(self.theta)
        if self.theta < np.pi:
            z = - self.step_penetration * np.sin(self.theta) - self.robot_height_offset
        else:
            z = - self.step_height * np.sin(self.theta) - self.robot_height_offset
        yaw = x/(desired_radius + self.lateral_leg_offset)
        y = self.lateral_leg_offset - (1 - np.cos(yaw)) * (desired_radius + self.lateral_leg_offset)
        return x, y, z, yaw
