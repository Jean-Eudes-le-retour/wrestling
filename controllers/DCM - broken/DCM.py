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
from controller import Robot
sys.path.append('..')
from utils.sensors import Accelerometer
from utils.fsm import Finite_state_machine
from utils.motion import Current_motion_manager, Motion_library
from ikpy.chain import Chain

IKPY_MAX_ITERATIONS = 100

from matplotlib import pyplot as plt
import numpy as np # numpy library for matrix computations
from FootTrajectoryGenerator import * # Foot trajectory generation Class
from DCMTrajectoryGenerator import DCMTrajectoryGenerator

#In this part we will specify the steps position and duration and we will implement foot trajectory generation
stepDuration = 0.5 #We select 1.2 second as step duration(step duration=SS+DS)    (1.2)
doubleSupportDuration = stepDuration/3 #We select 0.25 second as DS duration             (0.25)
pelvisHeight= 0.3 #Constant pelvis(CoM) height during walking                      (0.7)
maximumFootHeight = 0.02 #The maximum height of swing foot during each step        (0.07)

numberOfFootPrints = 17
FootPlanner = FootTrajectoryGenerator(stepDuration, doubleSupportDuration, maximumFootHeight, 0.5, numberOfFootPrints-2)
stepWidth = 0.025 #=(0.12) lateralDistanceOfFeet/2  
stepLength = 0.02 # (0.1) longitudinal distance between two sequential feet stepLength=stepStride/2, 
FootPrints = np.empty((numberOfFootPrints, 3))

#In the following we define the foot step positions(Ankle joint position projected on foot print)
for i in range(0,numberOfFootPrints):
    if(i%2==0):
        if(i==0):
            FootPrints[i][:]=[i*stepLength,stepWidth,0.0]
        elif(i==numberOfFootPrints-1):
            FootPrints[i][:]=[(i-2)*stepLength,stepWidth,0.0]            
        else:
            FootPrints[i][:]=[(i-1)*stepLength,stepWidth,0.0]
    else:
        FootPrints[i][:]=[(i-1)*stepLength,-stepWidth,0.0]
            

FootPlanner.setFootPrints(FootPrints)#We set the foot step positions
FootPlanner.generateTrajectory() #We generate the foot trajectory 
leftFootTrajectory = np.array(FootPlanner.getLeftFootTrajectory())
rightFootTrajectory = np.array(FootPlanner.getRightFootTrajectory())

CoPOffset=np.array([0.0,0.0]) #Offset between CoP and footprint position(Ankle position) 


DCMPlanner = DCMTrajectoryGenerator(0.02, pelvisHeight, stepDuration, doubleSupportDuration, numberOfFootPrints-3)#We create an object of foot DCMTrajectoryGenerator Class
CoPPositions=np.empty((DCMPlanner.numberOfSteps+1, 3))#Initialization of the CoP array

#In the following we define the foot steps positions
for i in range(0,DCMPlanner.numberOfSteps+1):
    if(i%2!=0):
        CoPPositions[i][:]=[(i)*stepLength-CoPOffset[0],stepWidth-CoPOffset[1],0.0]
        if(i==1):
            CoPPositions[i][:]=[(i)*stepLength,stepWidth-CoPOffset[1],0.0]
    else:
        CoPPositions[i][:]=[(i)*stepLength-CoPOffset[0],-stepWidth+CoPOffset[1],0.0]
        if(i==0):
            CoPPositions[i][:]=[(i)*stepLength,-stepWidth+CoPOffset[1],0.0]
DCMPlanner.setCoP(CoPPositions) # We set the desired CoP positions
DCMPlanner.setFootPrints(FootPrints) # We set the foot steps positions
DCMTrajectory = DCMPlanner.getDCMTrajectory()
initialCoM = np.array([0.0,0.0,DCMPlanner.CoMHeight])
comTrajectory = DCMPlanner.getCoMTrajectory(initialCoM)
DCMPlanner.calculateCoPTrajectory()

print("Loop iterations", int((DCMPlanner.numberOfSamplesPerSecond) * CoPPositions.shape[0] * DCMPlanner.stepDuration))
print("comTrajectory.shape",comTrajectory.shape)
print("leftFootTrajectory.shape",leftFootTrajectory.shape)
print("rightFootTrajectory.shape",rightFootTrajectory.shape)

class Eliot (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())
        self.i = 0
        # the Finite State Machine (FSM) is a way of representing a robot's behavior as a sequence of states
        self.fsm = Finite_state_machine(
            states=['DEFAULT', 'BLOCKING_MOTION', 'FRONT_FALL', 'BACK_FALL', 'END'],
            initial_state='DEFAULT',
            actions={
                'BLOCKING_MOTION': self.pending,
                'DEFAULT': self.walk,
                'FRONT_FALL': self.front_fall,
                'BACK_FALL': self.back_fall,
                'END': self.end
            }
        )

        # accelerometer
        self.accelerometer = Accelerometer(self.getDevice('accelerometer'), self.time_step)

        # there are 7 controllable LEDs on the NAO robot, but we will use only the ones in the eyes
        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        # Shoulder roll motors
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = Current_motion_manager()
        self.library = Motion_library()

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
                position_sensor.enable(self.time_step)
                self.L_leg_motors.append(motor)

        self.R_leg_motors = []
        for link in self.right_leg_chain.links:
            if link.name != 'Base link' and link.name != "RLeg_effector_fixedjoint":
                motor = self.getDevice(link.name)
                # motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.time_step)
                self.R_leg_motors.append(motor)
        
        # Inverse kinematics seed
        self.left_previous_joints = [0, 0, 0, -0.523, 1.047, -0.524, 0, 0]
        self.right_previous_joints = [0, 0, 0, -0.523, 1.047, -0.524, 0, 0]

    def run(self):
        self.leds['right'].set(0x0000ff)
        self.leds['left'].set(0x0000ff)
        self.current_motion.set(self.library.get('Stand'))
        self.fsm.transition_to('BLOCKING_MOTION')
        while self.step(self.time_step) != -1:
            #self.detect_fall()
            self.fsm.execute_action()
    
    def detect_fall(self):
        """Detect a fall and update the FSM state."""
        self.accelerometer.update_average()
        [acc_x, acc_y, _] = self.accelerometer.get_average()
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
            self.fsm.transition_to('DEFAULT')

    def walk(self):
        if self.i >= int((DCMPlanner.numberOfSamplesPerSecond) * CoPPositions.shape[0] * DCMPlanner.stepDuration):
            self.fsm.transition_to('END')
            return
        desired_position = [
                (rightFootTrajectory[self.i, 0] - comTrajectory[self.i, 0]),
                -(rightFootTrajectory[self.i, 1] - comTrajectory[self.i, 1]),
                rightFootTrajectory[self.i, 2] - 0.3
            ]
        print(desired_position)
        right_target_commands = self.right_leg_chain.inverse_kinematics(
            desired_position, # not working
            np.eye(3),
            initial_position=self.right_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(right_target_commands[1:], self.R_leg_motors):
            motor.setPosition(command)
        self.right_previous_joints = right_target_commands

        left_target_commands = self.left_leg_chain.inverse_kinematics(
            [
                (leftFootTrajectory[self.i, 0] - comTrajectory[self.i, 0]),
                -(leftFootTrajectory[self.i, 1] - comTrajectory[self.i, 1]),
                leftFootTrajectory[self.i, 2] - 0.3
            ],
            np.eye(3),
            initial_position=self.left_previous_joints,
            max_iter=IKPY_MAX_ITERATIONS,
            orientation_mode='all'
        )
        for command, motor in zip(left_target_commands[1:], self.L_leg_motors):
            motor.setPosition(command)
        self.left_previous_joints = left_target_commands
        self.i += 1

    def front_fall(self): 
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def back_fall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')
    
    def end(self):
        pass

if False:
    tempL=np.array(FootPlanner.lFoot_)
    tempR=np.array(FootPlanner.rFoot_)
    figg1, (ax1) = plt.subplots(figsize = (14, 7))
    ax1.plot(tempL[:,0],'r',label="Left")
    ax1.plot(tempR[:,0],'b',label="Right")
    ax1.set_xlabel('sampling time')
    ax1.set_ylabel('X Foot Position(m)')
    figg1.legend(bbox_to_anchor=(0.8, 0.8))
    plt.show()

    figg2, (ax3) = plt.subplots(figsize = (14, 7))
    ax3.plot(tempL[:,2],'red',label="Left")
    ax3.plot(tempR[:,2],'blue',label="Right")
    ax3.set_xlabel('sampling time')
    ax3.set_ylabel('Z Foot Position(m)')
    figg2.legend(bbox_to_anchor=(0.8, 0.8))
    plt.show() 

    figg3, (ax2) = plt.subplots(figsize = (14, 7))
    ax2.plot(tempL[:,1],'--r',label="Left")
    ax2.plot(tempR[:,1],'--b',label="Right")
    ax2.set_xlabel('sampling time')
    ax2.set_ylabel('Y Foot Position(m)')
    ax2.set(xlim=(-0.2, 4000), ylim=(-0.3, 0.3))
    figg3.legend(bbox_to_anchor=(0.8, 0.8))
    plt.show() 

    fig4, (ax4) = plt.subplots(figsize = (14, 7))
    footWidth=0.044
    footLength=0.03
    for i in range(0,np.size(DCMPlanner.footPrints,0)):
        rect = plt.Rectangle((DCMPlanner.footPrints[i,0]-footLength/2, DCMPlanner.footPrints[i,1]-footWidth/2), footLength, footWidth,
            facecolor="pink", alpha=0.4) 
        ax4.add_patch(rect)
    ax4.set(xlim=(-0.2, 0.4), ylim=(-0.4, 0.4))
    ax4.scatter(DCMPlanner.CoP[:,0],DCMPlanner.CoP[:,1],c='red',label="CoP")
    ax4.set_xlabel('X(m)')
    ax4.set_ylabel('Y(m)')
    ax4.plot(DCMPlanner.DCM[:,0],DCMPlanner.DCM[:,1],'blue',label="DCM")
    ax4.plot(DCMPlanner.CoM[:,0],DCMPlanner.CoM[:,1],'springgreen',label="CoM")
    fig4.legend(bbox_to_anchor=(0.87, 0.87))
    plt.show()


    figg5, (ax5) = plt.subplots(figsize = (14, 7))
    ax5.plot(DCMPlanner.DCM[:,0],'b',label="DCM")
    ax5.plot(DCMPlanner.CoM[:,0],'g',label="CoM")
    ax5.plot(DCMPlanner.CoPTrajectory[:,0],'--r',label="CoP")
    ax5.set_xlabel('sampling time')
    ax5.set_ylabel('X Trajectory(m)')
    ax5.set(xlim=(-0.0,2000), ylim=(-0.0, 1.4))
    figg5.legend(bbox_to_anchor=(0.8, 0.8))
    plt.show() 

    figg6, (ax6) = plt.subplots(figsize = (14, 7))
    ax6.plot(DCMPlanner.DCM[:,1],'b',label="DCM")
    ax6.plot(DCMPlanner.CoM[:,1],'g',label="CoM")
    ax6.plot(DCMPlanner.CoPTrajectory[:,1],'--r',label="CoP")
    ax6.set_ylabel('Y Trajectory(m)')
    ax6.set_xlabel('sampling time')
    ax6.set(xlim=(10,2000), ylim=(-0.14, 0.14))
    figg6.legend(bbox_to_anchor=(0.8, 0.8))
    plt.show()

    figg7, (ax7) = plt.subplots(figsize = (14, 7))
    ax7.plot(DCMPlanner.DCM[:,2],'b',label="DCM")
    ax7.plot(DCMPlanner.CoM[:,2],'g',label="CoM")
    ax7.set_ylabel('Z Trajectory(m)')
    ax7.set_xlabel('sampling time')
    ax7.set(xlim=(0,2000), ylim=(-0.1, 0.8))
    figg7.legend(bbox_to_anchor=(0.8, 0.8))
    plt.show()

# create the Robot instance and run main loop
wrestler = Eliot()
wrestler.run()
