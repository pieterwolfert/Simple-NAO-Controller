'''
======================================================================================================
 Written by REDSPHINX
 12 September 2014 
 version 1.1

 This is the SimpleSounds module. Everything that has to so with the speakers of the NAO will be here. 

 This module allows you to make the NAO:
 - Speak

 The required software version is naoqi 2.1
======================================================================================================
'''


from Config import Config
import time
from naoqi import ALProxy
import math
import almath
import re
import sys


class SimpleSounds:
    def __init__(self):
        self.talkProxy = ALProxy("ALTextToSpeech", ROBOT_IP, PORT)

    def speak(self, word):
        self.talkProxy.say(word)
