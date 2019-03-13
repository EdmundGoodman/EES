from time import sleep, time
from PIL import Image
import atexit
import cv2
import numpy as np
from PIL import Image
import math
import io
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
        GPIO.setup(26, GPIO.OUT, initial=1)
        GPIO.setup(27, GPIO.OUT, initial=1)
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

        def on_release(key):
        	self.stop()
        	if key == Key.esc:
                #ESCD2in.stopstop()
                self.stop()
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
        self.toggleGPIOPins(highPins=[26,27,8,11], lowPins=[16,19])

    def forward(self):
        self.stop()
        self.toggleGPIOPins(highPins=[], lowPins=[26,27,8,11,16,19])

    def turnLeft(self):
        self.stop()
        self.toggleGPIOPins(highPins=[8,11], lowPins=[26,27,16,19])

    def turnRight(self):
        self.stop()
        self.toggleGPIOPins(highPins=[26,27], lowPins=[8,11,16,19])

    def stop(self):
        self.toggleGPIOPins(highPins=list(range(14,20))+[8,11,26,27], lowPins=[])

    def takePhoto(self, resolution=(480, 360)):
        stream = io.BytesIO()
        with PiCamera() as camera:
            camera.resolution = resolution
            camera.capture(stream, "jpeg")
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(data, 1)
        return image

    def findBall(self, img, imgVerticalCentre, transforms, houghParams, display=False):
        def showImage(image, caption="Image"):
            if display:
                cv2.imshow(caption,image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        imgDimensions = img.shape
        cimg = img

        showImage(cimg, 'coloured circles')

        #Increase the colour contrast, and turn it grayscale
        clahe = cv2.createCLAHE(clipLimit=transforms["clipLimit"],
                                tileGridSize=transforms["tileGridSize"])
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l,a,b = cv2.split(lab)
        l2 = clahe.apply(l)
        lab = cv2.merge((l2,a,b))
        img = cv2.cvtColor(img, cv2.COLOR_LAB2BGR)
        showImage(img, 'clahe circles')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        showImage(img, 'grayscale circles')

        #Perform signal operations on the image to make it easier to analyses
        img = cv2.medianBlur(img, transforms["medianBlur1"])  # Remove noise before laplacian
        showImage(img, 'gray_blur circles')

        img = cv2.Laplacian(img, cv2.CV_8UC1, ksize=transforms["laplacian"])
        showImage(img, 'gray_lap circles')

        img = cv2.medianBlur(img, transforms["medianBlur2"]) # Further blur noise from laplacian
        showImage(img, 'processed circles')

        #https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghcircles
        #dp, the inverse ratio of the accumulator resolution to the image resolution
        #minDist, the minimum distance between the centers of the detected circles
        #param1, the higher threshold of the two passed to the Canny() edge detector
        #param2, the accumulator threshold for the circle centers at the detection stage.
        #        the smaller it is, the more false circles may be detected
        circles = cv2.HoughCircles(
            img,
            cv2.cv.CV_HOUGH_GRADIENT,
            dp=houghParams["dp"],
            minDist=houghParams["minDist"],
            param1=houghParams["param1"],
            param2=houghParams["param2"],
            minRadius=houghParams["minRadius"],
            maxRadius=houghParams["maxRadius"],
        )

        if circles is None:
            print("No circles found, try tuning the parameters")
            showImage('detected circles',cimg)
            return None

        #Find the closest circle (lowest vertical height - so heighest vertical pixel)
        #Then give direction you need to turn to centre it in the image
        circles = np.uint16(np.around(circles))[0]
        circles = np.uint16(sorted(circles,key=lambda l:l[1], reverse=True))

        if display:
            for i in circles:
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2) #Draw the outer circle
            cv2.line(cimg, (imgVerticalCentre, 0), (imgVerticalCentre, imgDimensions[1]), (255,0,0), 2)
            showImage('detected circles',cimg)

        closestCircle = circles[0]
        xCentralDisplacement = -1*(imgVerticalCentre - closestCircle[0])
        return xCentralDisplacement


    def turnToBall(self):
        #Note, we might need to crop the image to facilitate better characteristics
        imgVerticalCentre = None

        turnStepTime = 0.05
        turnTolerancePixels = 30
        timeForwardAfterTurn = 1
        noBallForwardTime = 1

        resolution = (480,360)
        transforms = {"clipLimit":3, "tileGridSize":(8,8),
            "medianBlur1":7, "laplacian":5, "medianBlur2":5}
        houghParams = {"dp":1,"minDist":100,"param1":50,
            "param2":65,"minRadius":30,"maxRadius":110,}
        display = True

        while True:
            data = raw_input("Step/Exit [*/x]: ")
            if data.lower() == "x":
                return False

            img = self.takePhoto(resolution)
            if imgVerticalCentre == None:
                imgVerticalCentre = int(img.shape[1]/2)
            turn = self.findBall(img, transforms, houghParams, display=True)
            print(turn)

            if turn is None:
                #Drive forwards
                print("No ball found")
                self.forward()
                sleep(noBallForwardTime)

            elif abs(turn) < turnTolerancePixels:
                #The ball is close to central, so drive forward to pick it up
                print("Found ball")
                #Spin up flywheels
                self.forward()
                sleep(timeForwardAfterTurn)
                #Spin down flywheels

            else:
                #Center the closest ball
                if turn < 0:
                    print("Turning left")
                    self.turnLeft()
                else:
                    print("Turning right")
                    self.turnRight()
                sleep(turnStepTime)

            self.stop() #Consider removing for cleaner runs


def main():
    robot = Robot()
    atexit.register(robot.shutdown)

    while True:
        data = raw_input("Remote control [r], Turn to ball [t]: ")
        if data == "r":
            robot.remoteControl()
        elif data == "t":
            robot.turnToBall()
        else:
            pass

if __name__ == "__main__":
    exit(main())
