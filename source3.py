from pivideostream2 import PiVideoStream
from time import sleep, time
from PIL import Image
import numpy as np
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
        self.setup()

    def setup(self):
        for i in range(14,20):
            GPIO.setup(i,GPIO.OUT,initial=1)
        GPIO.setup(8,GPIO.OUT,initial=1)
        GPIO.setup(11, GPIO.OUT,initial=1)
        GPIO.setup(26, GPIO.OUT, initial=1)
        GPIO.setup(27, GPIO.OUT, initial=1)
        self.ESCs = ESCD3in.PairESCController()
        #self.ESCs.calibrate()

    def shutdown(self):
        self.stop()
        GPIO.cleanup()
        self.ESCs.stopstop()
        print("Process Safely Stopped")

    def remoteControl(self):
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

        def on_release(key):
            self.stop()
            if key == Key.esc:
                self.ESCs.stop()
                return False

        with Listener(on_press=on_press,on_release=on_release) as listener:
                listener.join()

    def toggleGPIOPins(self, highPins, lowPins):
        for p in highPins:
            GPIO.output(p, GPIO.HIGH)
        for p in lowPins:
            GPIO.output(p, GPIO.LOW)

    def flyWheelsOn(self):
        self.ESCs.manual_drive("1300")

    def flyWheelsOff(self):
        self.ESCs.manual_drive("0")

    def backward(self):
        self.stop()
        self.toggleGPIOPins(highPins=[26,27,8,11], lowPins=[16,19])

    def forward(self):
        self.stop()
        self.toggleGPIOPins(highPins=[], lowPins=[26,27,8,11,16,19])

    def turnRight(self):
        self.stop()
        self.toggleGPIOPins(highPins=[8,11], lowPins=[26,27,16,19])

    def turnLeft(self):
        self.stop()
        self.toggleGPIOPins(highPins=[26,27], lowPins=[8,11,16,19])

    def stop(self):
        self.toggleGPIOPins(highPins=list(range(14,20))+[8,11,26,27], lowPins=[])


    def getBlocks(self):
        blocks = BlockArray(100)
        count = pixy.ccc_get_blocks(100, blocks)
        if count > 0:
            for index in range (0, count):
                centerx = blocks[index].m_x + blocks[index].m_width/2
                centery = blocks[index].m_y - blocks[index].m_height/2
            return 1,centerx, centery, blocks[index].m_width, blocks[index].m_height
        return None,None,None,None,None

    def turnToFace(self):
        try:
            u,x,y,width,height = self.getBlocks()
            if x is None:
                self.turnLeft()
                time.sleep(0.4)
                self.stop()
                time.sleep(1)
            if x > 180:
                self.turnRight()
                while x > 210:
                    u,x,y,width,height = self.getBlocks()
                self.stop()
            elif x < 180:
                self.turnLeft()
                while x < 150:
                    u,x,y,width,height = self.getBlocks()
                self.stop()
            return True
        except TypeError:
            return False

    def getBall(self):
        try:
            time.sleep(1)
            if not self.turnToFace():
                return False
            u,x,y,width,height = self.getBlocks()

            self.forward()
            time.sleep(0.5)
            self.flyWheelsOn()

            while x is not None:
                u,x,y,width,height = robot.getBlocks()
            time.sleep(1.7)
            robot.flyWheelsOff()

        except TypeError:
            robot.flyWheelsOff()
            robot.stop()
            print("Couldn't see a ball")
            time.sleep(1)
            return False

    def autonomous(self):
        while True:
            u,x,y,width,height = robot.getBlocks()
            if x is not None:
                self.stop()
                self.getBall()
            else:
                self.turnRight()


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
