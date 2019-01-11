from time import sleep, time
from PIL import Image
import atexit
import numpy
import math
import io


#Pi only libraries
try:
    import pigpiod

    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)

    from picamera import PiCamera
    camera = PiCamera()

    piOnlyLibraries = True
except ImportError:
    piOnlyLibraries = False



class Robot:
    def __init__(self):
        self.setup()

    def setup(self):
        for i in range(14,20):
            GPIO.setup(i,GPIO.OUT,initial=1)
        GPIO.setup(8,GPIO.OUT,initial=1)
        GPIO.setup(11, GPIO.OUT,initial=1)

        camera.resolution = (20,20)
        camera.framerate = 80

    def shutdown(self):
        self.stop()
        self.setFlyWheelSpeed(0)
        GPIO.cleanup()
        #camera.close()
        print("Process Safely Stopped")

    def remoteControl(self):
        while True:
            direction = input("Direction [aswd]: ").lower()
            directionFuncs = [self.turnLeft, self.backward, self.forward, self.turnRight]
            directionFuncs["aswd".index(direction)]()
            time.sleep(1)
            self.stop()

    def toggleGPIOPins(self, highPins, lowPins):
        for p in highPins:
            GPIO.output(p, GPIO.HIGH)
        for p in lowPins:
            GPIO.output(p, GPIO.LOW)

    def backward(self):
        self.stop()
        self.toggleGPIOPins(highPins=[14,15,8,11], lowPins=[16,19])

    def forward(self):
        self.stop()
        self.toggleGPIOPins(highPins=[], lowPins=[14,15,8,11,16,19])

    def turnLeft(self):
        self.stop()
        self.toggleGPIOPins(highPins=[8,11], lowPins=[14,15,16,19])

    def turnRight(self):
        self.stop()
        self.toggleGPIOPins(highPins=[14,15], lowPins=[8,11,16,19])

    def stop(self):
        self.toggleGPIOPins(highPins=list(range(14,20))+[8,11], lowPins=[])

    def setFlyWheelSpeed(self):
        pass

    def takePhoto(self):
        stream = io.BytesIO()
        camera.capture(stream, format="bmp", use_video_port=True)
        frame = Image.open(io.BytesIO(stream))

    def calibrateColour(self):
        frame = self.takePhoto()
        colour = (20,255,0)

    def turnToBall(self):
        ballColour = self.calibrateColour()

        #https://codereview.stackexchange.com/questions/184044/processing-an-image-to-extract-green-screen-mask

        #We want to mask the image to only show the green of the tennis ball
        #then find the centre of the ball (k-means, n=1?), and turn the wheels
        #to place it in the center line, then drive to it

        #Then return to the original course, unless another is seen


def main():
    #robot = Robot()
    #atexit.register(robot.shutdown)

    #robot.remoteControl()




if __name__ == "__main__":
    exit(main())
