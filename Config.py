'''
======================================================================================================
 Written by REDSPHINX
 12 September 2014 
 version 1.1

 This is a py file containing global final variables.

 The required software version is naoqi 2.1
======================================================================================================
'''
import math
#import almath

class Config:

    # Connection
    #ROBOT_IP =  "10.42.0.29" #David
    ROBOT_IP = "10.42.0.65" #Hal
    #ROBOT_IP = "10.42.0.53" #Daisy
    PORT = 9559


    # Conversions
    DEG2RAD = math.pi/180.0 # Convert Deg to Rad
    RAD2DEG = 180.0/math.pi # Convert Rad to Deg

    # Interface things
    FRAMEWIDTH = 300
    FRAMEHEIGHT = 500

    # Movement
    MAXSTEPSIZE = 8  # cm
    MINSTEPSIZE = 4  # cm
    MAXTHETA = 30  # in degrees CHANGED TO RADIANS IN CALCULATIONS
    MINTHETA = 10  #  in degrees CHANGED TO RADIANS IN CALCULATIONS
    UNIT = 4  # cm, the unit of distance in our case. so the robot moves in multiplicities of this unit
    THETAUNIT = 10
    RLEG = "RLeg"
    LLEG = "LLeg"
    SPEED = 1.0  # decrease to increase accuracy, robot will move slower though
    DIRECTIONS = ["L", "R", "Fw", "Bw"]


    # Vision
    CAMERA_H_FOV = 46.4 * DEG2RAD # Horizontal field of view
    CAMERA_V_FOV = 34.8 * DEG2RAD # Vertical field of view
    RESW = 320#640#320 #160.0 #Capture width
    RESH = 240#480#240 #120.0 #Capture height
    FOVHOR = 46.40 #"horizontal" field of view
    FOVVER = 34.80 #"vertical" field of view
