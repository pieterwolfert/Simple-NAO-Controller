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
        self.motionProxy = ALProxy("ALMotion", ROBOT_IP, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", ROBOT_IP, PORT)
        #self.talkProxy = ALProxy("ALTextToSpeech", ROBOT_IP, PORT) TODO make seperate sounds class
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
    DIRECTIONS = ["L", "R", "Fw", "Bw"]
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
                direction = DIRECTIONS[0]
                stepSize = stepSizeY
                for i in xrange(0, amountStepsY):
                    if i % 2 == 0:
                        self.setStep(LLEG, stepSizeX, stepSizeY, theta)
                        lastMovedLeg = LLEG
                    else:
                        self.setStep(RLEG, stepSizeX, stepSizeY, theta)
                        lastMovedLeg = RLEG
            else:
                positivity = False
                direction = DIRECTIONS[1]
                stepSize = -stepSizeY
                for i in xrange(0, amountStepsY):
                    if i % 2 == 0:
                        self.setStep(RLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = RLEG
                    else:
                        self.setStep(LLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = LLEG
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
                direction = DIRECTIONS[2]
                stepSize = stepSizeX
                for i in xrange(0, amountStepsX):
                    if i % 2 == 0:
                        self.setStep(RLEG, stepSizeX, stepSizeX, theta)
                        lastMovedLeg = RLEG
                    else:
                        self.setStep(LLEG, stepSizeX, stepSizeX, theta)
                        lastMovedLeg = LLEG
            else:
                positivity = False
                direction = DIRECTIONS[3]
                stepSize = -stepSizeX
                for i in xrange(0, amountStepsX):
                    if i % 2 == 0:
                        self.setStep(RLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = RLEG
                    else:
                        self.setStep(LLEG, -stepSizeX, -stepSizeY, theta)
                        lastMovedLeg = LLEG
            self.setLastStep(lastMovedLeg, direction, positivity, stepSize)
        else:
            print "error: either x or y input has to be 0"

        #self.standStraight()
        logObj.logWrite(time.time().__str__() + "_{0}_{1}_{2}_{3}_{4}".format(action, x, y, theta, SPEED))
        return [time.time().__str__(), action, x, y, theta, SPEED]
        pass

    # returns how many steps to take and with which step size
    # ! distance already converted to m in moveXYCm method.
    # ! distance here has to be int
    def getSteps(self, distance):
        distance = math.fabs(distance)
        if distance % MAXSTEPSIZE == 0:
            steps = distance/(UNIT*(MAXSTEPSIZE/MINSTEPSIZE))
            stepSize = MAXSTEPSIZE
        elif distance % MINSTEPSIZE == 0:
            steps = distance/UNIT
            stepSize = MINSTEPSIZE
        else:
            steps = 0
            stepSize = 0
            print("distance is not valid; must be a multiplication of ", UNIT)
            logObj.logWrite("distance is not valid; must be a multiplication of ", UNIT)
        return int(steps), stepSize

    # set a step with a speed
    # ! distance already converted to m in moveXYCm method
    def setStep(self, legName, X, Y, Theta):
        #print("setStep")
        legName = [legName]
        footSteps = [[X, Y, Theta]]
        fractionMaxSpeed = [SPEED]
        clearExisting = False
        self.motionProxy.setFootStepsWithSpeed(legName, footSteps, fractionMaxSpeed, clearExisting)
        #self.proxy.setFootSteps(legName, footSteps, timeList, clearExisting Don't use this

    # set the last step to complete the movement
    # DIRECTIONS = ["L", "R", "Fw", "Bw"]
    def setLastStep(self, lastMovedLeg, direction, positivity, stepSize):
        theta = 0
        if lastMovedLeg == LLEG:
            legToMove = RLEG
        elif lastMovedLeg == RLEG:
            legToMove = LLEG

        if direction == DIRECTIONS[0] or DIRECTIONS[1]:
            x = 0
            y = 0.1 # something with stepSize? no idea, since this moves the foot a distance relative to the other foot
        elif direction == DIRECTIONS[2] or DIRECTIONS[3]:
            x = 0.6 # something with stepSize? see above
            y = 0
        self.setStep(legToMove, x, y, theta)
        self.stand()
        #self.standStraight()

    # get the amount of steps needed to rotate amount of theta in. steps is how many steps the NAO needs to take to make the turn,
    # thetaSize is the size of theta in degrees of a turn in one step
    def getThetaSteps(self, theta):
        theta = float(theta)# math.fabs(float(theta))
        #if theta % MAXTHETA == 0:
        #    steps = theta/(THETAUNIT*(MAXTHETA/MINTHETA))*2 + 1
        #    thetaSize = MAXTHETA
        #el
        if theta % MINTHETA == 0:
            steps = theta/THETAUNIT
            thetaSize = MINTHETA
        else:
            steps = 0
            print("theta is not valid; must be a multiplication of " + MINTHETA)
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
            startLeg = [RLEG]
            otherLeg = [LLEG]
            thetaSize = -thetaSize*DEG2RAD
        else:
            startLeg = [LLEG]
            otherLeg = [RLEG]
            thetaSize = thetaSize*DEG2RAD

        #self.standStraight()
        self.motionProxy.walkInit()
        footSteps = [[0, 0, thetaSize]]
        fractionMaxSpeed = [SPEED]
        clearExisting = False

        steps = int(math.fabs(steps))
        for i in xrange(0, steps):
            if i % 2 == 0:
                self.motionProxy.setFootStepsWithSpeed(startLeg, footSteps, fractionMaxSpeed, clearExisting)
            else:
                self.motionProxy.setFootStepsWithSpeed(otherLeg, footSteps, fractionMaxSpeed, clearExisting)

        #  take last step?
        self.stand()
        logObj.logWrite(time.time().__str__() + "_{0}_{1}_{2}_{3}_{4}".format(action, x, y, theta, SPEED))
        theta = theta*DEG2RAD # sends theta in radians

        return [time.time().__str__(), action, x, y, theta, SPEED]


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

    def simpleKick(self):
        # TODO


#mot = Motion()

