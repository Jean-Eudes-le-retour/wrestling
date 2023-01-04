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

class Average():
    """Class that takes care of the computation of a list of values' running average."""

    def __init__(self, dimensions, history_steps=10):
        self.HISTORY_STEPS = history_steps
        if dimensions > 1:
            self.is_vector = True
            self.average = [0]*dimensions
            self.history = [[0]*dimensions]*self.HISTORY_STEPS
        else:
            self.is_vector = False
            self.average = 0
            self.history = [0]*self.HISTORY_STEPS

    def get_new_average(self, value):
        """Returns the current accelerometer average of the last HISTORY_STEPS values."""
        self.update_average(value)
        return self.average

    def update_average(self, value):
        """Updates the average with a new value."""
        self.history.pop(0)
        self.history.append(value)
        if self.is_vector:
            self.average = [sum(col)/self.HISTORY_STEPS for col in zip(*self.history)]
        else:
            self.average = sum(self.history)/self.HISTORY_STEPS

class Kinematics():
    """Class that takes care of the computation of the inverse kinematics."""
    # should I import this here or at the top of this file?
    from ikpy.chain import Chain
    import numpy as np

    IKPY_MAX_ITERATIONS = 4

    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step

        self.left_leg_chain = self.Chain.from_urdf_file(
            '../../protos/nao.urdf',
            base_elements=['base_link', 'LHipYawPitch'],
            active_links_mask=[False, True, True,
                               True, True, True, True, False]
        )

        self.right_leg_chain = self.Chain.from_urdf_file(
            '../../protos/nao.urdf',
            base_elements=['base_link', 'RHipYawPitch'],
            active_links_mask=[False, True, True,
                               True, True, True, True, False]
        )

        self.L_leg_motors = []
        for link in self.left_leg_chain.links:
            if link.name != 'Base link' and link.name != "LLeg_effector_fixedjoint":
                motor = robot.getDevice(link.name)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(time_step)
                self.L_leg_motors.append(motor)
        
        self.R_leg_motors = []
        for link in self.right_leg_chain.links:
            if link.name != 'Base link' and link.name != "RLeg_effector_fixedjoint":
                motor = robot.getDevice(link.name)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(time_step)
                self.R_leg_motors.append(motor)

        self.left_previous_joints = [0, 0, 0, -0.524, 1.047, -0.524, 0, 0]
        self.right_previous_joints = [0, 0, 0, -0.524, 1.047, -0.524, 0, 0]
    
    def ik_left_leg(self, target_position, target_orientation=np.eye(3)):
        """Computes the inverse kinematics of the left leg."""
        self.left_previous_joints = self.left_leg_chain.inverse_kinematics(
            target_position,
            target_orientation,
            initial_position=self.left_previous_joints,
            max_iter=self.IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        return self.left_previous_joints
    
    def ik_right_leg(self, target_position, target_orientation=np.eye(3)):
        """Computes the inverse kinematics of the right leg."""
        self.right_previous_joints = self.right_leg_chain.inverse_kinematics(
            target_position,
            target_orientation,
            initial_position=self.right_previous_joints,
            max_iter=self.IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        return self.right_previous_joints