#!/usr/bin/env python
import cv2
import numpy as np
from PIL import Image
import io
import time

from picamera import PiCamera


def getImage():
    stream = io.BytesIO()
    with PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        time.sleep(5)
        camera.capture(stream, "jpeg")
    data = np.fromstring(stream.getvalue(), dtype=np.uint8)
    image = cv2.imdecode(data, 1)
##    image = image[:,:,::-1]
    return image


def main():
##    img = cv2.imread('circle.jpg',0)


    prevTurn = 0
    while True:
        img = getImage()
        imgDimensions = img.shape
        imgVerticalCentre = int(img.shape[1]/2)
        
        imgVerticalCentre += prevTurn
        turn = turnToBall(img, imgVerticalCentre, turnSize=10, display=True)
        if turn != prevTurn and prevTurn != 0:
            break
        else:
            prevTurn = turn


def turnToBall(img, imgVerticalCentre=None, turnSize=10, display=True):
    print(imgVerticalCentre)

    if display:
        cv2.imshow('coloured circles',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    imgDimensions = img.shape
    cimg = img
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
##    cimg = img

    if display:
        cv2.imshow('initial circles',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    #Perform signal operations on the image to make it easier to analyses

    #img = cv2.equalizeHist(img) #Increase saturation of the image
    gray_blur = cv2.medianBlur(img, 7)  # Remove noise before laplacian
    gray_lap = cv2.Laplacian(gray_blur, cv2.CV_8UC1, ksize=5)
    dilate_lap = cv2.dilate(gray_lap, (3, 3))  # Fill in gaps from blurring. This helps to detect circles with broken edges.
    lap_blur = cv2.bilateralFilter(dilate_lap, 5, 9, 9) # Furthur remove noise introduced by laplacian. This removes false pos in space between the two groups of circles.
    out_blur = cv2.medianBlur(lap_blur, 5) # Further blur noise from laplacian
    img = out_blur

    if display:
        cv2.imshow('processed circles',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    #https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghcircles
    #Tune param2 to remove false positives
    #Tune min & max radius to the possible ball sizes
    circles = cv2.HoughCircles(
        img,
        cv2.cv.CV_HOUGH_GRADIENT,
        dp=1, #Inverse ratio of the accumulator resolution to the image resolution
        minDist=100, #Minimum distance between the centers of the detected circles
        param1=50, #the higher threshold of the two passed to the Canny() edge detector
        param2=90, #the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected
        minRadius=10,
        maxRadius=150,
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
    xCentralDisplacement = imgVerticalCentre - closestCircle[0]
    if xCentralDisplacement < 0:
        return xCentralDisplacement #turnSize
    else:
        return xCentralDisplacement #-1*turnSize


main()
