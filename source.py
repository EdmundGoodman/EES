from time import sleep, time
from PIL import Image
import atexit
import cv2
import numpy as np
from PIL import Image
import math
import io
#from ESCD2in import *
import ESCD2in

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
        #ESCD2in.calibrate()
    def shutdown(self):
        self.stop()
        GPIO.cleanup()
        ESCD2in.stopstop()
        print("Process Safely Stopped")

    def remoteControl(self):
        from pynput.keyboard import Key, Listener
        def on_press(key):
        	if key == Key.down:
        		self.backward()
        	if key == Key.up:
        		self.forward()
        	if key == Key.right:
        		self.turnRight()
        	if key == Key.left:
        		self.turnLeft()
                #if key == Key.q:
                #    robot.flyWheelsOn()
                #if key == Key.e:
                #    robot.flyWheelsOff()

        def on_release(key):
        	self.stop()
        	if key == Key.esc:
                #ESCD2in.stopstop()
         		return False

        with Listener(on_press=on_press,on_release=on_release) as listener:
        		listener.join()

    def toggleGPIOPins(self, highPins, lowPins):
        for p in highPins:
            GPIO.output(p, GPIO.HIGH)
        for p in lowPins:
            GPIO.output(p, GPIO.LOW)

    def flyWheelsOn(self):
        ESCD2in.manual_drive("1400")

    def flyWheelsOff(self):
        ESCD2in.manual_drive("0")

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


    def takePhoto(self, resolution=(480, 360)):
        stream = io.BytesIO()
        with PiCamera() as camera:
            camera.resolution = resolution
            camera.capture(stream, "jpeg")
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(data, 1)
        return image

    def findBall(self, img, imgVerticalCentre=None, display=True):

        if display:
            cv2.imshow('coloured circles',img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        imgDimensions = img.shape
        cimg = img

    ##    Increase the colour contrast

        clahe = cv2.createCLAHE(clipLimit=3, tileGridSize=(8,8))
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l,a,b = cv2.split(lab)
        l2 = clahe.apply(l)
        lab = cv2.merge((l2,a,b))
        img = cv2.cvtColor(img, cv2.COLOR_LAB2BGR)

        if display:
            cv2.imshow('clahe circles',img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if display:
            cv2.imshow('initial circles',img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


        #Perform signal operations on the image to make it easier to analyses

        #img = cv2.equalizeHist(img) #Increase saturation of the image
        gray_blur = cv2.medianBlur(img, 7)  # Remove noise before laplacian
        if display:
            cv2.imshow('gray_blur circles',gray_blur)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        gray_lap = cv2.Laplacian(gray_blur, cv2.CV_8UC1, ksize=5)
        if display:
            cv2.imshow('gray_lap circles',gray_lap)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        '''dilate_lap = cv2.dilate(gray_lap, (10, 10))  # Fill in gaps from blurring. This helps to detect circles with broken edges.
        if display:
            cv2.imshow('dilate_lap circles',dilate_lap)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        lap_blur = cv2.bilateralFilter(dilate_lap, 5, 9, 9) # Furthur remove noise introduced by laplacian. This removes false pos in space between the two groups of circles.
        if display:
            cv2.imshow('lap_blur circles',lap_blur)
            cv2.waitKey(0)
            cv2.destroyAllWindows()'''

        lap_blur = gray_lap

        out_blur = cv2.medianBlur(lap_blur, 5) # Further blur noise from laplacian
        if display:
            cv2.imshow('processed circles',out_blur)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

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
            param2=65, #the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected
            minRadius=30,
            maxRadius=110,
        )

        if circles is None:
            print("No circles found, try tuning the parameters")
            exit()


        #Find the closest circle (lowest vertical height - so heighest vertical pixel)
        #Then give direction you need to turn to centre it in the image
        circles = np.uint16(np.around(circles))[0]
        circles = np.uint16(sorted(circles,key=lambda l:l[1], reverse=True))

        if display:
            for i in circles:
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2) #Draw the outer circle
            cv2.line(cimg, (imgVerticalCentre, 0), (imgVerticalCentre, imgDimensions[1]), (255,0,0), 2)

            cv2.imshow('detected circles',cimg)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        closestCircle = circles[0]
        xCentralDisplacement = -1*(imgVerticalCentre - closestCircle[0])
        return xCentralDisplacement



    def turnToBall(self):
        #Note, we might need to crop the image to facilitate better characteristics

        scalePhoto = self.takePhoto()
        imgVerticalCentre = int(scalePhoto.shape[1]/2)

        turnStepTime = 0.05
        turnTolerancePixels = 30
        timeForwardAfterTurn = 1
        noBallForwardTime = 1

        #self.stop()
        while True:
            data = raw_input("Step/Exit [*/x]: ")
            if data.lower() == "x":
                return False

            img = self.takePhoto()
            turn = self.findBall(img, imgVerticalCentre)

            if turn is None:
                #Drive forwards
                #self.forward()
                sleep(noBallForwardTime)
            elif abs(turn) < turnTolerancePixels:
                #The ball is close to cental, so drive forward to pick it up
                print("Found ball")
                #self.forward()
                sleep(timeForwardAfterTurn)
            else:
                #Center the closest ball
                print("Turning to ball")
                if turn < 0:
                    pass #self.turnLeft()
                else:
                    pass #self.turnRight()
                sleep(turnStepTime)
            #self.stop()


def main():

    robot = Robot()
    atexit.register(robot.shutdown)


    while True:
        data = raw_input("Remote control [r], Turn to ball [t]: ")
        if data == "r":
            pass #robot.remoteControl()
        elif data == "t":
            robot.turnToBall()
        else:
            pass

if __name__ == "__main__":
    exit(main())
