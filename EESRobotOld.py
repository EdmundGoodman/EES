import RPi.GPIO as GPIO
import atexit
GPIO.setmode(GPIO.BCM)
class Robot:
    def __init__(self):
        for i in range(14,20):
            GPIO.setup(i,GPIO.OUT,initial = 1)
        GPIO.setup(8,GPIO.OUT,initial = 1)
        GPIO.setup(11, GPIO.OUT, initial = 1)

    def Back(self):
        self.Stop()
        GPIO.output(14, GPIO.HIGH)
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(11, GPIO.HIGH)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    def Forward(self):
        self.Stop()
        GPIO.output(14, GPIO.LOW)
        GPIO.output(15,GPIO.LOW)
        GPIO.output(8, GPIO.LOW)
        GPIO.output(11, GPIO.LOW)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19,GPIO.LOW)

    def TurnLeft(self):
        self.Stop()
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(11,GPIO.HIGH)
        GPIO.output(14, GPIO.LOW)
        GPIO.output(15, GPIO.LOW)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    def TurnRight(self):
        self.Stop()
        GPIO.output(8, GPIO.LOW)#LOW to HIGH 14 TO 17
        GPIO.output(11,GPIO.LOW)# LOW to HIGH 15 TO 18
        GPIO.output(14, GPIO.HIGH)
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(16, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

    def Stop(self):
        for i in range(14,20):
            GPIO.output(i,GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
        GPIO.output(11, GPIO.HIGH)

    def Shutdown(self):
        GPIO.cleanup()
        print("Process Safely Stopped")
robot = Robot()
atexit.register(robot.Shutdown)
