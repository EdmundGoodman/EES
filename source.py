from time import sleep, time
from PIL import Image
import atexit
import cv2
import numpy as np
from PIL import Image
import math
import io


import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

from picamera import PiCamera




class Robot:
    def __init__(self):
        self.setup()

    def setup(self):
        for i in range(14,20):
            GPIO.setup(i,GPIO.OUT,initial=1)
        GPIO.setup(8,GPIO.OUT,initial=1)
        GPIO.setup(11, GPIO.OUT,initial=1)

    def shutdown(self):
        self.stop()
        self.setFlyWheelSpeed(0)
        GPIO.cleanup()
        print("Process Safely Stopped")

    def remoteControl(self):
        while True:
            direction = raw_input("Direction [aswd]: ").lower()
            directionFuncs = [self.turnLeft, self.backward, self.forward, self.turnRight]
            directionFuncs["aswd".index(direction)]()
            sleep(1)
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

    def setFlyWheelSpeed(self, speed):
        pass

    def takePhoto(self):
        stream = io.BytesIO()
        with PiCamera() as camera:
            camera.resolution = (480, 360)
            camera.capture(stream, "jpeg")
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(data, 1)
        return image

    def findBall(self, img, imgVerticalCentre=None):
        #Returns the number of pixels a ball is from the center line

        imgDimensions = img.shape
        cimg = img

        #Increase the colour contrast
        clahe = cv2.createCLAHE(clipLimit=3, tileGridSize=(8,8))
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l,a,b = cv2.split(lab)
        l2 = clahe.apply(l)
        lab = cv2.merge((l2,a,b))
        img = cv2.cvtColor(img, cv2.COLOR_LAB2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Perform signal operations on the image to make it easier to analyses
        gray_blur = cv2.medianBlur(img, 13)  # Remove noise before laplacian
        gray_lap = cv2.Laplacian(gray_blur, cv2.CV_8UC1, ksize=5)
        out_blur = cv2.medianBlur(gray_lap, 3) # Further blur noise from laplacian
        img = out_blur

        #https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghcircles
        #Tune param2 to remove false positives
        #Tune min & max radius to the possible ball sizes
        circles = cv2.HoughCircles(
            img,
            cv2.cv.CV_HOUGH_GRADIENT,
            dp=1, #Inverse ratio of the accumulator resolution to the image resolution
            minDist=100, #Minimum distance between the centers of the detected circles
            param1=50, #the higher threshold of the two passed to the Canny() edge detector
            param2=80, #the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected
            minRadius=10,
            maxRadius=110,
        )

        if circles is None:
            print("No circles found, try tuning the parameters")
            return None

        #Find the closest circle (lowest vertical height - so heighest vertical pixel)
        #Then give direction you need to turn to centre it in the image
        circles = np.uint16(np.around(circles))[0]
        circles = np.uint16(sorted(circles,key=lambda l:l[1], reverse=True))

        closestCircle = circles[0]
        xCentralDisplacement = -1*(imgVerticalCentre - closestCircle[0])
        return xCentralDisplacement



    def turnToBall(self):
        #Note, we might need to crop the image to facilitate better characteristics

        scalePhoto = self.takePhoto()
        imgVerticalCentre = int(img.shape[1]/2)

        turnStepTime = 0.5
        turnTolerancePixels = 30
        timeForwardAfterTurn = 1
        noBallForwardTime = 1

        while True:
            img = self.takePhoto()
            turn = self.findBall(img, imgVerticalCentre)
            self.stop()
            if turn is None:
                #Drive forwards
                self.forward()
                time.sleep(noBallForwardTime)
            elif abs(turn) < turnTolerancePixels:
                #The ball is close to cental, so drive forward to pick it up
                self.forward()
                time.sleep(timeForwardAfterTurn)
            else:
                #Center the closest ball
                if turn < 0:
                    self.turnLeft()
                else:
                    self.turnRight()
                time.sleep(turnStepTime)


def main():
    robot = Robot()
    atexit.register(robot.shutdown)

    robot.remoteControl()
    robot.turnToBall()

if __name__ == "__main__":
    exit(main())
