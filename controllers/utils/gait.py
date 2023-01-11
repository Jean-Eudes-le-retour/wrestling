# Copyright 1996-2023 Cyberbotics Ltd.
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

import numpy as np
from . import kinematics

class Ellipsoid_gait_generator():
    """Simple gait generator, based on an ellipsoid path.
    Derived from chapter 2 in paper:
    G. Endo, J. Morimoto, T. Matsubara, J. Nakanishi, and G. Cheng,
    “Learning CPG-based Biped Locomotion with a Policy Gradient Method: Application to a Humanoid Robot,”
    The International Journal of Robotics Research, vol. 27, no. 2, pp. 213-228,
    Feb. 2008, doi: 10.1177/0278364907084980.
    """
    MAX_STEP_LENGTH_FRONT = 0.045
    MAX_STEP_LENGTH_SIDE = 0.02
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
        self.step_period = 0.5 # time to complete one step
        self.step_length_front = self.MAX_STEP_LENGTH_FRONT # distance traveled by the feet in one step
        self.step_length_side = self.MAX_STEP_LENGTH_SIDE # distance traveled by the feet in one step
        self.step_twist = 0.3 
        self.step_height = 0.04 # height of the ellipsoid path
        self.step_penetration = 0.005 # depth of the ellipsoid path
        self.calibration_factor = 0.93
    
    def update_theta(self):
        """Update the angle of the ellipsoid path and clip it to [-pi, pi]"""
        self.theta = -(2 * np.pi * self.robot.getTime() / self.step_period) % (2 * np.pi) - np.pi

    def compute_leg_position(self, is_right, desired_radius=1e3, heading_angle=0):
        """Compute the desired positions of a leg for a desired radius (R > 0 is a right turn)."""
        # TODO: clip position to possible range?
        desired_radius *= self.calibration_factor # actual radius is bigger than the desired one, so we "correct" it
        factor = 1 if is_right else -1 # the math is the same for both legs, except for some signs
        if abs(desired_radius) > 0.1:
            amplitude_x = self.adapt_step_length(heading_angle) * (desired_radius - factor * self.lateral_leg_offset) / desired_radius
            x = factor * amplitude_x * np.cos(self.theta)
            yaw = - x/(desired_radius - factor * self.lateral_leg_offset)
            y = - (1 - np.cos(yaw)) * (desired_radius - factor * self.lateral_leg_offset)
        else:
            # rotate in place
            rotate_right = -1 if desired_radius > 0 else 1
            x = rotate_right * self.adapt_step_length(heading_angle) * np.cos(self.theta)
            yaw = rotate_right * factor * self.step_twist * np.cos(self.theta)
            y = (1 - np.cos(yaw)) * (factor * self.lateral_leg_offset)
        if heading_angle != 0:
            x, y = rotate(x, y, heading_angle)
        y += - factor * self.lateral_leg_offset
        amplitude_z = self.step_penetration if factor * self.theta < 0 else self.step_height
        # vestibulospinal reflex: corrects the robot's roll
        amplitude_z += factor * self.imu.getRollPitchYaw()[0] * self.roll_reflex_factor
        # extensor response: pushes on the leg when it is on the ground
        force_values = self.right_foot_sensor.getValues() if is_right else self.left_foot_sensor.getValues()
        force_magnitude = np.linalg.norm(np.array([force_values[0], force_values[1], force_values[2]]))
        if force_magnitude > 5:
            amplitude_z += self.force_reflex_factor * force_magnitude
        z = factor * amplitude_z * np.sin(self.theta) - self.robot_height_offset
            
        return x, y, z, yaw
    
    def adapt_step_length(self, heading_angle):
        """Adapt the step length to the heading angle."""
        # need to bring the heading angle fron [-pi, pi] to [0, pi/2]
        if heading_angle < 0:
            heading_angle = - heading_angle
        if heading_angle > np.pi/2:
            heading_angle = np.pi - heading_angle
        factor = heading_angle / (np.pi/2)
        amplitude = self.step_length_front * (1 - factor) + self.step_length_side * factor
        return amplitude
    
    def set_step_amplitude(self, amount):
        """Set the amplitude of the step. amount is between 0 and 1."""
        self.step_length_front = self.MAX_STEP_LENGTH_FRONT * amount
        self.step_length_side = self.MAX_STEP_LENGTH_SIDE * amount

class Gait_manager():
    """Connects the Kinematics class and the Ellipsoid_gait_generator class together to have a simple gait interface."""
    def __init__(self, robot, time_step):
        self.time_step = time_step
        self.gait_generator = Ellipsoid_gait_generator(robot, self.time_step)
        joints = ['HipYawPitch', 'HipRoll', 'HipPitch', 'KneePitch', 'AnklePitch', 'AnkleRoll']
        self.L_leg_motors = []
        for joint in joints:
            motor = robot.getDevice(f'L{joint}')
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(time_step)
            self.L_leg_motors.append(motor)
        
        self.R_leg_motors = []
        for joint in joints:
            motor = robot.getDevice(f'R{joint}')
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(time_step)
            self.R_leg_motors.append(motor)
    
    def update_theta(self):
        self.gait_generator.update_theta()
    
    def command_to_motors(self, desired_radius=1e3, heading_angle=0):
        """
        Compute the desired positions of the robot's legs for a desired radius (R > 0 is a right turn) and a desired heading angle (in radians).
        Send the commands to the motors.
        """
        x, y, z, yaw = self.gait_generator.compute_leg_position(is_right=True, desired_radius=desired_radius, heading_angle=heading_angle)
        right_target_commands = kinematics.inverse_leg(x*1e3, y*1e3, z*1e3, 0, 0, yaw, is_left=False)
        for command, motor in zip(right_target_commands, self.R_leg_motors):
            motor.setPosition(command)

        x, y, z, yaw = self.gait_generator.compute_leg_position(is_right=False, desired_radius=desired_radius, heading_angle=heading_angle)
        left_target_commands = kinematics.inverse_leg(x*1e3, y*1e3, z*1e3, 0, 0, yaw, is_left=True)
        for command, motor in zip(left_target_commands, self.L_leg_motors):
            motor.setPosition(command)

def rotate(x, y, angle):
    """Rotate a point by a given angle."""
    return x * np.cos(angle) - y * np.sin(angle), x * np.sin(angle) + y * np.cos(angle)
