'''
======================================================================================================
 Written by REDSPHINX
 12 September 2014 
 version 1.1

 This is the SimpleMotions module. Everything that has to do with the movement of the NAO will be here.

 This module allows you to make the NAO:
 - Walk
 - Move it's head
 - Perform a kicking manoeuvre
 - Stand up
 - Lie down
 - Unstiffen
 - Stiffen

 The required software version is naoqi 2.1
 =====================================================================================================
'''


import time
from naoqi import ALProxy
import math
import almath
import re
import sys

from Config import Config

class SimpleMotions:
    def __init__(self):
        self.motionProxy = ALProxy("ALMotion", Config.ROBOT_IP, Config.PORT)
        self.postureProxy = ALProxy("ALRobotPosture", Config.ROBOT_IP, Config.PORT)
        robotConfig = self.motionProxy.getRobotConfig()
        # print the robot configuration information
        for i in range(len(robotConfig[0])):
            print robotConfig[0][i], ": ", robotConfig[1][i]

    # turn on stiffness of body
    def stiffnessOn(self, motionProxy):
        allJoints = "Body"
        pStiffnessLists = 1.0
        pTimeLists = 1.0
        motionProxy.stiffnessInterpolation(allJoints, pStiffnessLists, pTimeLists)

    # turn off stiffness
    def stiffnessOff(self, motionProxy):
        allJoints = "Body"
        pStiffnessLists = 0.0
        pTimeLists = 1.0
        motionProxy.stiffnessInterpolation(allJoints, pStiffnessLists, pTimeLists)

    # make robot stand up
    def stand(self):  # def stand(self, name, speed):
        self.stiffnessOn(motionProxy=self.motionProxy)
        self.postureProxy.goToPosture("Stand", 0.4)

    # make the robot stand up fast
    def fastStand(self):  # def stand(self, name, speed):
        self.stiffnessOn(motionProxy=self.motionProxy)
        self.postureProxy.goToPosture("Stand", 1)

    # make the robot sit
    def sit(self):
        self.stiffnessOn(motionProxy=self.motionProxy)
        self.postureProxy.goToPosture("SitRelax", 1.0)

    '''
    make the robot move in a direction
    x: positive move forward, negative move backwards [-1.0 to 1.0]
    y: positive left, negative right [-1.0 to 1.0]
    theta: positive for counterclockwise, negative for clockwise [-1.0 to 1.0]
    speed: determines the frequency of the steps, so the velocity [0.0 to 1.0]
    '''
    def move(self, x, y, theta, speed):
        x = float(x)
        y = float(y)
        theta = float(theta)
        speed = float(speed)
        self.motionProxy.setWalkTargetVelocity(x, y, theta, speed)
        logObj.logWrite(time.time().__str__() + "_3_{0}_{1}_{2}_{3}".format(x,y,theta,speed))

    '''
    as an alternative and more controllable method of moving,
    this moves the robot a desired amount of cm in units of 4 cm. This method makes the robot move L R Fw Bw
    for rotation see the rotateTheta() method
    Config.DIRECTIONS = ["L", "R", "Fw", "Bw"]
    sends as output [time,action,dForwards,dSideways,dtheta,speed]

    input given in integer cm
    '''
    # TODO figure out how many m one footstep is, for all cases so L R Bw Fw
    def moveXYCm(self, x, y):
        self.stand()
        # convert from input string to integer
        x = int(x)
        y = int(y)
        action = 1
        theta = 0
        #self.standStraight()
        self.motionProxy.walkInit()

        # pos is L, neg is R
        if x == 0:
            amountStepsY, stepSizeY = self.getSteps(y)
            #stepSizeY from cm to m
            stepSizeY = float(stepSizeY)  # / 100 apparently not necessary
            amountStepsX = 0
            stepSizeX = 0
            if y > 0:
                positivity = True
                direction = Config.DIRECTIONS[0]
                stepSize = stepSizeY
                for i in xrange(0, amountStepsY):
                    if i % 2 == 0:
                        self.setStep(Config.LLEG, stepSizeX, stepSizeY, theta)
                        lastMovedLeg = Config.LLEG
                    else:
                        self.setStep(Config.RLEG, stepSizeX, stepSizeY, theta)
                        lastMovedLeg = Config.RLEG
            else:
                positivity = False
                direction = Config.DIRECTIONS[1]
                stepSize = -stepSizeY
                for i in xrange(0, amountStepsY):
                    if i % 2 == 0:
                        self.setStep(Config.RLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = Config.RLEG
                    else:
                        self.setStep(Config.LLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = Config.LLEG
            self.setLastStep(lastMovedLeg, direction, positivity, stepSize)

        # pos is Fw, neg is Bw
        elif y == 0:
            amountStepsX, stepSizeX = self.getSteps(x)
            # convert from cm to m
            stepSizeX = float(stepSizeX) / 100
            amountStepsY = 0
            stepSizeY = 0
            if x > 0:
                positivity = True
                direction = Config.DIRECTIONS[2]
                stepSize = stepSizeX
                for i in xrange(0, amountStepsX):
                    if i % 2 == 0:
                        self.setStep(Config.RLEG, stepSizeX, stepSizeX, theta)
                        lastMovedLeg = Config.RLEG
                    else:
                        self.setStep(Config.LLEG, stepSizeX, stepSizeX, theta)
                        lastMovedLeg = Config.LLEG
            else:
                positivity = False
                direction = Config.DIRECTIONS[3]
                stepSize = -stepSizeX
                for i in xrange(0, amountStepsX):
                    if i % 2 == 0:
                        self.setStep(Config.RLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = Config.RLEG
                    else:
                        self.setStep(Config.LLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = Config.LLEG
            self.setLastStep(lastMovedLeg, direction, positivity, stepSize)
        else:
            print "error: either x or y input has to be 0"

        #self.standStraight()
        logObj.logWrite(time.time().__str__() + "_{0}_{1}_{2}_{3}_{4}".format(action, x, y, theta, Config.SPEED))
        return [time.time().__str__(), action, x, y, theta, Config.SPEED]
        pass

    # returns how many steps to take and with which step size
    # ! distance already converted to m in moveXYCm method.
    # ! distance here has to be int
    def getSteps(self, distance):
        distance = math.fabs(distance)
        if distance % Config.MAXSTEPSIZE == 0:
            steps = distance/(Config.UNIT*(Config.MAXSTEPSIZE/Config.MINSTEPSIZE))
            stepSize = Config.MAXSTEPSIZE
        elif distance % Config.MINSTEPSIZE == 0:
            steps = distance/Config.UNIT
            stepSize = Config.MINSTEPSIZE
        else:
            steps = 0
            stepSize = 0
            print("distance is not valid; must be a multiplication of ", Config.UNIT)
            logObj.logWrite("distance is not valid; must be a multiplication of ", Config.UNIT)
        return int(steps), stepSize

    # set a step with a speed
    # ! distance already converted to m in moveXYCm method
    def setStep(self, legName, X, Y, Theta):
        #print("setStep")
        legName = [legName]
        footSteps = [[X, Y, Theta]]
        fractionMaxSpeed = [Config.SPEED]
        clearExisting = False
        self.motionProxy.setFootStepsWithSpeed(legName, footSteps, fractionMaxSpeed, clearExisting)
        #self.proxy.setFootSteps(legName, footSteps, timeList, clearExisting Don't use this

    # set the last step to complete the movement
    # Config.DIRECTIONS = ["L", "R", "Fw", "Bw"]
    def setLastStep(self, lastMovedLeg, direction, positivity, stepSize):
        theta = 0
        if lastMovedLeg == Config.LLEG:
            legToMove = Config.RLEG
        elif lastMovedLeg == Config.RLEG:
            legToMove = Config.LLEG

        if direction == Config.DIRECTIONS[0] or Config.DIRECTIONS[1]:
            x = 0
            y = 0.1 # something with stepSize? no idea, since this moves the foot a distance relative to the other foot
        elif direction == Config.DIRECTIONS[2] or Config.DIRECTIONS[3]:
            x = 0.6 # something with stepSize? see above
            y = 0
        self.setStep(legToMove, x, y, theta)
        self.stand()
        #self.standStraight()

    # get the amount of steps needed to rotate amount of theta in. steps is how many steps the NAO needs to take to make the turn,
    # thetaSize is the size of theta in degrees of a turn in one step
    def getThetaSteps(self, theta):
        theta = float(theta)# math.fabs(float(theta))
        #if theta % Config.MAXTHETA == 0:
        #    steps = theta/(Config.THETAUNIT*(Config.MAXTHETA/Config.MINTHETA))*2 + 1
        #    thetaSize = Config.MAXTHETA
        #el
        if theta % Config.MINTHETA == 0:
            steps = theta/THETAConfig.UNIT
            thetaSize = Config.MINTHETA
        else:
            steps = 0
            print("theta is not valid; must be a multiplication of " + Config.MINTHETA)
        return int(steps), thetaSize

    # rotate an n amount of theta in degrees.
    # one turn step = 29.9656927 deg or 0.523 radians
    # theta: positive for counterclockwise, negative for clockwise [-1.0 to 1.0]
    # action code = 2
    def rotateTheta(self, theta):
        self.stand()
        theta = int(theta)
        action = 2
        x = 0
        y = 0
        steps, thetaSize = self.getThetaSteps(theta)
        if theta < 0:
            startLeg = [Config.RLEG]
            otherLeg = [Config.LLEG]
            thetaSize = -thetaSize*DEG2RAD
        else:
            startLeg = [Config.LLEG]
            otherLeg = [Config.RLEG]
            thetaSize = thetaSize*DEG2RAD

        #self.standStraight()
        self.motionProxy.walkInit()
        footSteps = [[0, 0, thetaSize]]
        fractionMaxSpeed = [Config.SPEED]
        clearExisting = False

        steps = int(math.fabs(steps))
        for i in xrange(0, steps):
            if i % 2 == 0:
                self.motionProxy.setFootStepsWithSpeed(startLeg, footSteps, fractionMaxSpeed, clearExisting)
            else:
                self.motionProxy.setFootStepsWithSpeed(otherLeg, footSteps, fractionMaxSpeed, clearExisting)

        #  take last step?
        self.stand()
        logObj.logWrite(time.time().__str__() + "_{0}_{1}_{2}_{3}_{4}".format(action, x, y, theta, Config.SPEED))
        theta = theta*DEG2RAD # sends theta in radians

        return [time.time().__str__(), action, x, y, theta, Config.SPEED]


    # stop the walking gracefully
    def stop(self):
        #self.motionProxy.stopMove()
        self.motionProxy.setWalkTargetVelocity(0.0, 0.0, 0.0, 0.0)
        self.stand()
        #logObj.logWrite(time.time().__str__() + "_4_0_0_0_0")

    # TODO put this in the sounds class
    #def talk(self, word):
        #self.talkProxy.say(word)

    def moveHeadPitch(self, theta, speed):
        theta = float(theta)
        speed = float(speed)
        self.motionProxy.setAngles("HeadPitch", theta, 0.1)

    def chillOut(self):
        self.stiffnessOn(motionProxy=self.motionProxy)
        self.postureProxy.goToPosture("LyingBack", 1.0)
        self.stiffnessOff(motionProxy=self.motionProxy)

    # one of Roel's things
    #def measureAngle(self):
        self.stand()
        #name = "HeadPitch"
        #c = self.motionProxy.getAngles(name, True)
        #return 90.0 - (180.0/math.pi)*c[0]

    def correctHipPitch(self):
        names = ['LHipPitch', 'RHipPitch']
        angles = [0, 0]
        fractionMaxSpeed = 0.1
        self.motionProxy.setAngles(names, angles, fractionMaxSpeed)

    def simpleKickRight(self):
        # TODO
        names = ['RShoulderRoll', 'RShoulderPitch', 'LShoulderRoll', 'LShoulderPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        angles = [[-0.3], [0.4], [0.5], [1.0], [0.0], [-0.4, -0.2], [0.95, 1.5], [-0.55, -1], [0.2], [0.0], [-0.4], [0.95], [-0.55], [0.2]]
        times =  [[ 0.5], [0.5], [0.5], [0.5], [0.5], [ 0.4,  0.8], [ 0.4, 0.8],  [0.4, 0.8], [0.4], [0.5], [ 0.4], [ 0.4], [ 0.4],  [0.4]]
        self.motionProxy.angleInterpolation(names, angles, times, True)

        self.motionProxy.angleInterpolationWithSpeed(['RShoulderPitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch'], [1.5, -0.7, 1.05, -0.5], 1.0 , True)

        self.motionProxy.angleInterpolation(['RHipPitch', 'RKneePitch', 'RAnklePitch'], [-0.5, 1.1, -0.65], [[0.25], [0.25], [0.25]], True)
        self.normalPose(True) #TODO
        pass

    def simpleKickLeft(self):
        names = ['LShoulderRoll', 'LShoulderPitch', 'RShoulderRoll', 'RShoulderPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        angles = [[0.3], [0.4], [-0.5], [1.0], [0.0], [-0.4, -0.2], [0.95, 1.5], [-0.55, -1], [-0.2], [0.0], [-0.4], [0.95], [-0.55], [-0.2]]
        times =  [[0.5], [0.5], [0.5], [0.5], [0.5], [0.4, 0.8], [ 0.4, 0.8], [ 0.4, 0.8], [0.4], [0.5], [ 0.4], [0.4], [0.4], [0.4]]        
        self.motionProxy.angleInterpolation(names, angles, times, True)
        motionProxy.angleInterpolationWithSpeed(['LShoulderPitch', 'LHipPitch', 'LKneePitch', 'LAnklePitch'], [1.5, -0.7, 1.05, -0.5], 1.0, True)
        self.motionProxy.angleInterpolation(['LHipPitch', 'LKneePitch', 'LAnklePitch'],[-0.5, 1.1, -0.65], [[0.25], [0.25], [0.25]], True)    
        self.normalPose(True) #TODO
        pass

    def simpleKick(self):
        angle = 4.0
        self.motionProxy.setAngles('RShoulderPitch', 2, 1)
        self.motionProxy.setAngles(['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch'],
                [0.75*angle, 0.1, -0.1 -0.25*angle , -0.2 + -0.25 * angle, 0, 0.1 - 0.075*angle], 0.9)
        time.sleep(0.4 + 0.3 * angle)
        # return to start position
        self.motionProxy.angleInterpolation(['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch',
            'LAnkleRoll', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
            [[0], [0],[-0.4], [0.95], [-0.55],[0.15], [0], [-0.4], [0.95], [-0.55], [0.15]],
            [[0.65],[0.85],[0.85],[0.85],[0.85],[0.85],[0.85],[0.85],[0.85],[0.65],[0.75]], True)
        time.sleep(0.2)
        self.motionProxy.setAngles(['LShoulderRoll', 'LShoulderPitch', 'RShoulderRoll', 'RShoulderPitch'], [0, 1.2, 0, 1.2], 0.4)
        self.motionProxy.setAngles(['LAnkleRoll', 'RAnkleRoll'], [0, 0], 0.05)

# soft kick towards right, left leg
    def sideLeftKick(self):
        self.motionProxy.angleInterpolation(['RAnkleRoll', 'LAnkleRoll'], [-0.2, -0.2], [[0.4], [0.4]], True)

        names = list()
        angles = list()
        times = list()
        
        names = ['RShoulderRoll','LHipRoll', 'LHipPitch', 'LKneePitch','LAnklePitch','LAnkleRoll','RHipRoll','RHipPitch','RKneePitch','RAnklePitch','RAnkleRoll']
        angles = [[-0.4], [0.0, 0.3], [-0.4, -0.8],[0.95, 0.2], [-0.55, 0.6], [-0.25,-0.3],[0.05 ], [-0.4], [0.95], [-0.5],       [-0.25]]
        times  = [[0.2],          [0.3, 0.75],[0.5,   1.0],[0.5,  1.0], [ 0.5,  1.0], [0.5,   1.0],[ 1.0],    [0.5],      [0.5 ],      [ 0.5],       [ 0.5]]
        
        self.motionProxy.angleInterpolation(names, angles, times, True)
        
        self.motionProxy.angleInterpolation('LHipRoll', [-0.05], [0.1], True)
        time.sleep(0.1)
        self.motionProxy.angleInterpolation(['LHipRoll','LHipPitch','LKneePitch','LAnklePitch'], 
                                     [[0.0],    [-0.55],    [1.2],       [-0.6]], 
                                     [[0.5],    [0.6],      [0.6],       [0.6]], True)
        #self.normalPose(True)

