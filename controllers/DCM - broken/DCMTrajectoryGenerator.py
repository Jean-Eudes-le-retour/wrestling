import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

class DCMTrajectoryGenerator:
    def __init__(self, time_step, pelvisHeight, stepTiming, doubleSupportTime, numberOfFootSteps):
        self.time_step = time_step
        self.CoMHeight = pelvisHeight # We assume that CoM and pelvis are the same point
        self.stepDuration = stepTiming
        self.dsTime = doubleSupportTime
        self.numberOfSamplesPerSecond =  1/time_step #Number of sampling of the trajectory in each second
        self.numberOfSteps = numberOfFootSteps #This is the desired number of steps for walking
        self.alpha = 0.5 # We have 0<alpha<1 that is used for double support simulation
        self.DCM = list("")
        self.omega = math.sqrt(9.81/self.CoMHeight) #Omega is a constant value and is called natural frequency of linear inverted pendulum
        pass


    def getDCMTrajectory(self):
        self.findFinalDCMPositionsForEachStep() #or we can have another name for this function based on equation (8) of the jupyter notebook: for example findInitialDCMPositionOfEachStep()
        self.planDCMForSingleSupport() #Plan preliminary DCM trajectory (DCM without considering double support
        self.findBoundryConditionsOfDCMDoubleSupport() #Find the boundary conditions for double support 
        self.embedDoubleSupportToDCMTrajectory() #Do interpolation for double support and embed double support phase trajectory to the preliminary trajectory 
        return self.DCM


    def getCoMTrajectory(self,com_ini):
        #This class generates the CoM trajectory by integration of CoM velocity(that has been found by the DCM values)
        self.CoM = np.zeros_like(self.DCM)
        self.CoMDot = np.zeros_like(self.DCM)
        self.CoM[0] = com_ini
        self.CoMDot[0] = 0
        for kk in range(0,self.CoM.shape[0]-1):
            self.CoMDot[kk + 1] = self.omega*(self.DCM[kk] - self.CoM[kk]); #equation (3) in jupyter notebook
            self.CoM[kk + 1] = self.CoM[kk] + self.CoMDot[kk + 1]*self.time_step; #Simple euler numerical integration
            self.CoM[kk + 1][2] = self.CoMHeight
        return self.CoM


    def setCoP(self, CoP):
        self.CoP = CoP #setting CoP positions. Note: The CoP has an offset with footprint positions 
        pass

    def setFootPrints(self,footPrints):
        self.footPrints = footPrints #setting footprint positions. Note: The footprint has an offset with CoP positions 
        pass

    def findFinalDCMPositionsForEachStep(self): #Finding Final(=initial for previous, refer to equation 8) dcm for a step
        self.DCMForEndOfStep = np.copy(self.CoP) #initialization for having same shape
        self.DCMForEndOfStep[-1] = self.CoP[-1] #capturability constraint(3rd item of jupyter notebook steps for DCM motion planning section)

        for index in range(np.size(self.CoP,0)-2,-1,-1):
            self.DCMForEndOfStep[index] = self.CoP[index + 1] + \
                (self.DCMForEndOfStep[index + 1] - self.CoP[index + 1])*\
                math.exp(-self.omega*self.stepDuration) #equation 7 of the jupyter notebook
        pass

    def calculateCoPTrajectory(self):
        self.DCMVelocity = np.zeros_like(self.DCM)
        self.CoPTrajectory = np.zeros_like(self.DCM)
        self.DCMVelocity[0] = 0
        self.CoPTrajectory[0] = self.CoP[0]
        for kk in range(0,self.CoM.shape[0]-1):
            self.DCMVelocity[kk + 1] = (self.DCM[kk + 1] - self.DCM[kk])/self.time_step; #Numerical differentiation for solving DCM Velocity
            self.CoPTrajectory[kk + 1] = self.DCM[kk + 1] - self.DCMVelocity[kk + 1]/self.omega; #Use equation (10) to find CoP by having DCM and DCM Velocity
        pass


    def planDCMForSingleSupport(self): #The output of this function is a DCM vector with a size of (int(self.numberOfSamplesPerSecond* self.stepDuration * self.CoP.shape[0])) that is number of sample points for whole time of walking
        for iter in range(int(self.numberOfSamplesPerSecond* self.stepDuration * self.CoP.shape[0])):# We iterate on the whole simulation control cycles:  
            time = iter/self.numberOfSamplesPerSecond; #Finding the time of a corresponding control cycle
            i = math.floor(iter/(self.numberOfSamplesPerSecond*self.stepDuration)); #Finding the number of corresponding step of walking
            t = time - self.stepDuration*i; #The “internal” step time t is reset at the beginning of each step
            self.DCM.append(self.CoP[i] + (self.DCMForEndOfStep[i] - self.CoP[i])*math.exp(self.omega*(t - self.stepDuration))) #Use equation (9) for finding the DCM trajectory
        pass


    def findBoundryConditionsOfDCMDoubleSupport(self):
        self.initialDCMForDS = np.zeros((np.size(self.CoP,0),3))
        self.finalDCMForDS = np.zeros((np.size(self.CoP,0),3))
        self.initialDCMVelocityForDS = np.zeros((np.size(self.CoP,0),3))
        self.finalDCMVelocityForDS = np.zeros((np.size(self.CoP,0),3))
        for stepNumber in range(np.size(self.CoP,0)):
            if stepNumber == 0: #Boundary conditions of double support for the first step(equation 11b and 12b in Jupyter notebook)
                self.initialDCMForDS[stepNumber] = self.DCM[0]; #At the first step the initial dcm for double support is equal to the general initial DCM position, use (11b)
                self.finalDCMForDS[stepNumber] = self.CoP[0] + (self.DCM[0] - self.CoP[0])*math.exp(self.omega*(1 - self.alpha)*self.dsTime); # use (12b)
                self.initialDCMVelocityForDS[stepNumber] = self.omega*(self.initialDCMForDS[0] - self.CoP[0]); #You can find DCM velocity at each time by having DCM position for that time and the corresponding CoP position, see equation (4)
                self.finalDCMVelocityForDS[stepNumber] = self.omega*(self.finalDCMForDS[0] - self.CoP[0]); #You can find DCM velocity at each time by having DCM position for that time and the corresponding CoP position, see euqation (4))
            else: #Boundary conditions of double support for all steps except first step((equation 11 and 12 in Jupyter notebook))
                self.initialDCMForDS[stepNumber] = self.CoP[stepNumber - 1] + (self.DCMForEndOfStep[stepNumber - 1] - self.CoP[stepNumber - 1])*math.exp(-self.omega*self.alpha*self.dsTime); #use equation(11)
                self.finalDCMForDS[stepNumber] = self.CoP[stepNumber] + (self.DCMForEndOfStep[stepNumber - 1] - self.CoP[stepNumber])*math.exp(self.omega*(1 - self.alpha)*self.dsTime); #use equation(12)
                self.initialDCMVelocityForDS[stepNumber] = self.omega*(self.initialDCMForDS[stepNumber] - self.CoP[stepNumber-1]); #You can find DCM velocity at each time by having DCM position for that time and the corresponding CoP position, see euqation (4)
                self.finalDCMVelocityForDS[stepNumber] = self.omega*(self.finalDCMForDS[stepNumber] - self.CoP[stepNumber]); #You can find DCM velocity at each time by having DCM position for that time and the corresponding CoP position, see euqation (4)
            pass
    

    def doInterpolationForDoubleSupport(self,initialDCMForDS, finalDCMForDS, initialDCMVelocityForDS, finalDCMVelocityForDS,dsTime):
        #The implementation of equation (15) of Jupyter Notebook
        a = 2/(dsTime**3)*initialDCMForDS + 1/(dsTime**2)*initialDCMVelocityForDS - 2/(dsTime**3)*finalDCMForDS + 1/(dsTime**2)*finalDCMVelocityForDS; #first element of P matrix
        b = -3/(dsTime**2)*initialDCMForDS - 2/dsTime*initialDCMVelocityForDS + 3/(dsTime**2)*finalDCMForDS - 1/dsTime*finalDCMVelocityForDS; #second element of P matrix
        c = initialDCMVelocityForDS; #third element of P matrix
        d = initialDCMForDS; #fourth element of P matrix
        return a, b, c, d # a b c and are the elements of the P in equation (15)
    


    def embedDoubleSupportToDCMTrajectory(self): #Calculate and replace DCM position for double support with the corresponding time window of preliminary single support phase
        doubleSupportInterpolationCoefficients = list('')
        for stepNumber in range(np.size(self.CoP,0)):
            if(stepNumber==0):
                doubleSupportInterpolationCoefficients.append(self.doInterpolationForDoubleSupport(self.initialDCMForDS[stepNumber],self.finalDCMForDS[stepNumber],self.initialDCMVelocityForDS[stepNumber],self.finalDCMVelocityForDS[stepNumber],(1 - self.alpha)*self.dsTime)); #Create a vector of DCM Coeffient by using the doInterpolationForDoubleSupport function. Note that the double support duration for first step is not the same as other steps 
            else:
                doubleSupportInterpolationCoefficients.append(self.doInterpolationForDoubleSupport(self.initialDCMForDS[stepNumber],self.finalDCMForDS[stepNumber],self.initialDCMVelocityForDS[stepNumber],self.finalDCMVelocityForDS[stepNumber],self.dsTime))        
        #In the following part we will find the list of double support trajectories for all steps of walking
        listOfDoubleSupportTrajectories = list('')
        for stepNumber in range(np.size(self.CoP,0)):
            a, b, c, d = doubleSupportInterpolationCoefficients[stepNumber]; #use doubleSupportInterpolationCoefficients vector
            if(stepNumber==0): #notice double support duration is not the same as other steps
                doubleSupportTrajectory = np.zeros((int((1 - self.alpha)*self.dsTime*(1/self.time_step)),3))
                for t in range(int((1 - self.alpha)*self.dsTime*(1/self.time_step))):
                    tt = t*self.time_step;
                    doubleSupportTrajectory[t] = a*tt**3 + b*tt**2 + c*tt + d; #use equation 16 (only the DCM position component)
                listOfDoubleSupportTrajectories.append(doubleSupportTrajectory)
            else:
                doubleSupportTrajectory = np.zeros((int(self.dsTime*(1/self.time_step)),3))
                for t in range(int(self.dsTime*(1/self.time_step))):
                    tt = t*self.time_step;
                    doubleSupportTrajectory[t] = a*tt**3 + b*tt**2 + c*tt + d; #use equation 16 (only the DCM position component)
                listOfDoubleSupportTrajectories.append(doubleSupportTrajectory)      

        #In the following part we will replace the double support trajectories for the corresponding double support time-window  in the preliminary DCM trajectory
        DCMCompleteTrajectory = np.array(self.DCM)#First we put preliminary DCM trajectory into a new array and in th following we will replace the double support part 
        
        for stepNumber in range(self.CoP.shape[0]):
            if stepNumber == 0:
                #the first step starts with double support and notice double support duration is not the same as other steps
                DCMCompleteTrajectory[0:int((1 - self.alpha)*self.dsTime/self.time_step)] = listOfDoubleSupportTrajectories[stepNumber][:]#fill the corresponding interval for DCM index for double support part
            else: 
                start = stepNumber*int(self.stepDuration/self.time_step) - int(self.alpha*self.dsTime/self.time_step);
                end = start + int(self.dsTime*(1/self.time_step));
                DCMCompleteTrajectory[start:end] = listOfDoubleSupportTrajectories[stepNumber][:]

        self.l = listOfDoubleSupportTrajectories;
        self.oldDCM = np.array(self.DCM);
        self.DCM = DCMCompleteTrajectory
        temp = np.array(self.DCM)

        pass
