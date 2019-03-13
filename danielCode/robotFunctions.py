import RPi.GPIO as GPIO
import atexit
GPIO.setmode(GPIO.BCM)
from time import sleep
import pigpio
from ESCD2in import *
import ESCD2in
class Robot:
    def __init__(self):
        for i in range(14,20):
            GPIO.setup(i,GPIO.OUT,initial = 1)
        GPIO.setup(8,GPIO.OUT,initial = 1)
        GPIO.setup(26, GPIO.OUT, initial = 1)
        GPIO.setup(11, GPIO.OUT, initial = 1)
        GPIO.setup(27, GPIO.OUT, initial= 1)
        #ESCD2in.calibrate()

    def Back(self):
        self.Stop()
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(11, GPIO.HIGH)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    def Forward(self):
        self.Stop()
        GPIO.output(26, GPIO.LOW)
        GPIO.output(27,GPIO.LOW)
        GPIO.output(8, GPIO.LOW)
        GPIO.output(11, GPIO.LOW)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19,GPIO.LOW)

    def TurnRight(self):
        self.Stop()
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(11,GPIO.HIGH)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    def TurnLeft(self):
        self.Stop()
        GPIO.output(8, GPIO.LOW)#LOW to HIGH 14 TO 17
        GPIO.output(11,GPIO.LOW)# LOW to HIGH 15 TO 18
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    def Stop(self):
        for i in range(14,20):
            GPIO.output(i,GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(11, GPIO.HIGH)
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(27, GPIO.HIGH)
    def FlyWheelsOn(self):
        ESCD2in.manual_drive("2000")
    def FlyWheelsOff(self):
        ESCD2in.manual_drive("0")

    def Shutdown(self):
        GPIO.cleanup()
        print("Process Safely Stopped")
        ESCD2in.stopstop()

robot = Robot()
atexit.register(robot.Shutdown)
