from pivideostream2 import PiVideoStream
from time import sleep, time
from PIL import Image
import numpy as np
import random
import atexit
import math
import cv2
import io
import os

from picamera.array import PiRGBArray
from imutils.video import FPS
from picamera import PiCamera
import imutils

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

import ESCD3in

import pixy
from ctypes import *
from pixy import *

import VL53L1X

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

class Robot:
    def __init__(self):
        """Initialise the robot, by setting it up, and performing any other
        necessary procedures"""
        self.setup()

    def setup(self):
        """Initialise all the GPIO pins, and create an ESC & a TOF object"""
        for i in range(14,20):
            GPIO.setup(i,GPIO.OUT,initial=1)
        GPIO.setup(8,GPIO.OUT,initial=1)
        GPIO.setup(11, GPIO.OUT,initial=1)
        GPIO.setup(26, GPIO.OUT, initial=1)
        GPIO.setup(27, GPIO.OUT, initial=1)
        self.ESCs = ESCD3in.PairESCController()

        self.tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        self.tof.open()
        self.tof.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

    def shutdown(self):
        """Fully shutdown the robot, i.e. powering of the motors, the ESCs,
        the TOF, and clean up to GPIO pins"""
        self.stop()
        self.ESCs.stopstop()
        self.tof.stop_ranging()
        GPIO.cleanup()
        print("Process Safely Stopped")

    def remoteControl(self):
        """Remote control the robot from the command line"""
        from pynput.keyboard import Key, Listener
        self.flag = False

        def on_press(key):
            if key == Key.down:
                self.backward()
            if key == Key.up:
                self.forward()
            if key == Key.right:
                self.turnRight()
            if key == Key.left:
                self.turnLeft()
            if key == Key.space:
                if self.flag == False:
                    self.flyWheelsOn()
                    self.flag = True
                else:
                    self.flyWheelsOff()
                    self.flag = False
            if key == Key.tab:
                print(self.getDistance())

        def on_release(key):
            self.stop()
            if key == Key.esc:
                self.ESCs.stop()
                return False

        with Listener(on_press=on_press,on_release=on_release) as listener:
                listener.join()

    def toggleGPIOPins(self, highPins, lowPins):
        """Toggle the given GPIO pins
        Parameter 1: highPins [list]; set pins in this list to a high logical value
        Parameter 2: lowPins [list]; set pins in this list to low logical value"""
        for p in highPins:
            GPIO.output(p, GPIO.HIGH)
        for p in lowPins:
            GPIO.output(p, GPIO.LOW)

    def flyWheelsOn(self, duty="1300"):
        """Set the duty of the ESCs to a given value
        Parameter 1: duty [string]; set the duty of both ESCs to this value"""
        self.ESCs.manual_drive(str(duty))

    def flyWheelsOff(self):
        """Set the duty of the ESCs to 0 - i.e. turn them off"""
        self.ESCs.manual_drive("0")

    def backward(self):
        """Drive the robot backwards"""
        self.stop()
        self.toggleGPIOPins(highPins=[26,27,8,11], lowPins=[16,13])

    def forward(self):
        """Drive the robot forwards"""
        self.stop()
        self.toggleGPIOPins(highPins=[], lowPins=[26,27,8,11,16,13])

    def turnRight(self):
        """Turn the robot right"""
        self.stop()
        self.toggleGPIOPins(highPins=[8,11], lowPins=[26,27,16,13])

    def turnLeft(self):
        """Turn the robot left"""
        self.stop()
        self.toggleGPIOPins(highPins=[26,27], lowPins=[8,11,16,13])

    def stop(self):
        """Stop the robot"""
        self.toggleGPIOPins(highPins=list(range(13,20))+[8,11,26,27], lowPins=[])

    def getDistance(self):
        """Get the distance from the TOF sensor to the nearest obstacle
        Return 1: distance [int]; the distance to the nearest obstacle"""
        return self.tof.get_distance()

    def getBlocks(self):
        """Get various information about the most prominent circle in the image
        Return 1: centrex [int]; the x centre of the nearest circle
        Return 2: centrey [int]; the y centre of the nearest circle
        Return 3: blockWidth [int];
        Return 4: blockHeight [int];
        """
        blocks = BlockArray(100)
        count = pixy.ccc_get_blocks(100, blocks)
        if count > 0:
            for index in range (0, count):
                centerx = blocks[index].m_x + blocks[index].m_width/2
                centery = blocks[index].m_y - blocks[index].m_height/2
            return 1,centerx, centery, blocks[index].m_width, blocks[index].m_height
        return None,None,None,None,None

    def autonomousOld(self):
        """Autonomously collect balls
        """
        while True:
            u,x,y,width,height = robot.getBlocks()
            if x is not None:
                if x < 260 and x > 150: #If the ball is central
                    #print(y)
                    self.stop()
                    self.forward()
                    sleep(0.5)
                    self.flyWheelsOn()
                    sleep(2)
                    self.flyWheelsOff()
                    self.stop()
            else:
                self.turnRight()

    def isAboutToCrash(self, threshold=1000):
        """Check if the distance between a robot and a detected obstacle
        is below a threshold
        Return 1: answer [boolean]; is the robot about to crash"""
        return self.getDistance()<threshold

    def avoidWall(self, direction=None):
        """Turn in a random (or specified) direction to avoid the wall,
        then keep turning for another random period of time
        Parameter 1: direction [int] (optional); the direction in which to turn
        """
        direction = random.choice([0,1]) if direction is None else direction

        while isAboutToCrash():
            if direction == 0:
                self.turnRight()
            else:
                self.turnLeft()

        latencyPeriod = random.uniform(0.2,1.5)
        sleep(latencyPeriod)

        self.stop()

    def autonomous(self):
        """Autonomously collect balls
        Continuously drive forward, unless:
         - you see a ball, in which case turn to pick it up
         - you are too close to a wall, in case turn randomly to avoid it
        """
        while True:
            if self.isAboutToCrash():
                self.avoidWall()

            u,x,y,width,height = robot.getBlocks()
            if x is not None:
                if x < 260 and x > 150: #If the ball is central, collect it
                    self.stop()
                    self.flyWheelsOn()
                    #If the the ball is too close, wait to spin up the flywheels
                    if y < 100:
                        sleep(0.5)
                    #Drive forward until the ball is out of view for at least 0.5 seconds
                    self.forward()
                    while x is not None:
                        u,x,y,width,height = robot.getBlocks()
                        if self.isAboutToCrash():
                            #Stop driving forward, and defer the avoidance to the
                            #beginning of the loop
                            self.stop()
                            continue
                    sleep(0.5)
                    self.flyWheelsOff()
                    self.stop()
                elif x < 150: #If the ball is to the right, turn right
                    self.turnRight()
                elif x > 260: #If the ball is to the left, turn left
                    self.turnLeft()
            else:
                self.forward()


def main():
    robot = Robot()
    atexit.register(robot.shutdown)

    while True:
        os.system('clear')
        data = input("Remote control [r], Calibrate [c] Autonomous [a] or Exit [x]: ").lower()
        if data == "r":
            robot.remoteControl()
        if data == "c":
            robot.ESCs.calibrate()
        elif data == "t":
            robot.autonomous()
        elif data == "x":
            exit()
        else:
            pass

if __name__ == "__main__":
    exit(main())
