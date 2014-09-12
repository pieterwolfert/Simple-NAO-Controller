'''
======================================================================================================
 Written by REDSPHINX
 12 September 2014 
 version 1.1

 This is a simple controller with an interface for the NAO that allows you to quickly take control of:
 - The limbs
 - The head
 - The camera
 - The speaker

 The required software version is naoqi 2.1
======================================================================================================
'''

from Tkinter import *
from Config import Config
from SimpleMotions import SimpleMotions
from SimpleVisions import SimpleVisions
from SimpleSounds import SimpleSounds


root = Tk()
root.wm_title("Simple NAO Controller v1.1")
root.configure(background='white')
frame = Frame(root, frameWidth, frameHeight)
motionObj = SimpleMotions()
visionObj = SimpleVisions()
soundObj = SimpleSounds()

class SimpleControllerBeta:
    def __init__(self):
        frame.pack_propagate(0)
        self.createbuttons() 
        frame.pack()
        root.mainloop
        pass

    def createButtons(self):
        standUpButton = Button( frame, 
                                text = "Stand Up", 
                                background = "green",
                                foreground = "black",
                                command = lambda : self.wrapper(motionObj.fastStand()))
        standUpButton.pack()

        moveXYButton = Button(  frame, 
                                text = "Walk X/Y cm", 
                                background = "green", 
                                foreground = "black", 
                                command = lambda : self.wrapper(motionObj.moveXYCm(self.moveX.get(), self.moveY.get())))
        moveXYButton.pack()

        self.makeXEntry()
        self.makeYEntry()

        rotateButton = Button(  frame, 
                                text = "Rotate in Degrees", 
                                background = "green", 
                                foreground = "black", 
                                command = lambda : self.wrapper(motionObj.rotateTheta(self.moveTh.get())))
        rotateButton.pack()

        self.makeRotationThetaEntry()

        stopButton = Button(    frame, 
                                text = "STOP!", 
                                background = "red", 
                                foreground = "white", 
                                command = lambda : self.wrapper(motionObj.stop()))
        stopButton.pack()

        takePictureButton = Button( frame, 
                                    text = "Take a Picture", 
                                    background = "blue", 
                                    foreground = "black", 
                                    command = lambda : self.wrapper(visionObj.takePicture(self.name.get())))
        takePictureButton.pack()

        self.makePictureNameEntry()

        speakButton = Button(   frame, 
                                text = "Speak", 
                                background = "blue", 
                                foreground = "black", 
                                command = lambda : self.wrapper(soundObj.speak(self.message.get())))
        speakButton.pack()

        self.makeMessageEntry()

        moveHeadPitchButton = Button(   frame, 
                                        text = "Move Head Pitch", 
                                        background = "green",
                                        foreground = "black", 
                                        command = lambda : self.wrapper(motionObj.moveHeadPitch(self.headPitchTheta.get(), self.headPitchSpeed.get())))
        moveHeadPitchButton.pack()

        self.makeHeadPitchThetaEntry()
        self.makeHeadPitchSpeedEntry()

        chillOutButton = Button(    frame,
                                    text = "Chill Out", 
                                    background = "pink",
                                    foreground = "black", 
                                    command = lambda : self.wrapper(motionObj.chillOut()))
        chillOutButton.pack()

        kickButton = Button(    frame,
                                text = "Kick",
                                background = "green",
                                foreground = "black",
                                command = lambda : self.wrapper(motionObj.simpleKick()))
        kickButton.pack()

    def makeXEntry(self):
        self.moveX = Entry(frame)
        self.moveX.pack()
        self.moveX.delete(0, END)
        self.moveX.insert(0, "enter in cm")
        pass

    def makeYEntry(self):
        self.moveY = Entry(frame)
        self.moveY.pack()
        self.moveY.delete(0, END)
        self.moveY.insert(0, "enter in cm")
        pass

    def makeRotationThetaEntry(self):
        self.moveTh = Entry(frame)
        self.moveTh.pack()
        self.moveTh.delete(0, END)
        self.moveTh.insert(0, "enter in degrees")
        pass

    def makePictureNameEntry(self):
        self.name = Entry(frame)
        self.name.pack()
        self.name.delete(0, END)
        self.name.insert(0, ".jpg")
        pass

    def makeMessageEntry(self):
        self.chirp = Entry(frame)
        self.chirp.pack()
        self.chirp.delete(0, END)
        self.chirp.insert(0, "Hello! I am a NAO")
        pass

    def wrapper(self, func):
        func
        self.update()
        pass

    def makeHeadPitchThetaEntry(self):
            self.headPitchTheta = Entry(frame)
            self.headPitchTheta.pack()
            self.headPitchTheta.delete(0, END)
            self.headPitchTheta.insert(0, "0.3")
            pass

    def makeHeadPitchSpeedEntry(self):
            self.headPitchSpeed = Entry(frame)
            self.headPitchSpeed.pack()
            self.headPitchSpeed.delete(0, END)
            self.headPitchSpeed.insert(0, "0.5")
            pass

NAOControl = NAOControl()
