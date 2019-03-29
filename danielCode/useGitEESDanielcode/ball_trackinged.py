# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import io
import picamera.array
from PIL import Image
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (26, 86, 6)
greenUpper = (55, 255, 255)


# if a video path was not supplied, grab the reference
# to the webcam
# allow the camera or video file to warm up
time.sleep(2.0)



# keep looping
def DoStuff(frame):
	# grab the current frame
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball


	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			print(center)
	# update the points queue
	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	

cam = picamera.PiCamera()
time.sleep(2)
stream = io.BytesIO()
count = 0
'''cam.contrast = (-5)
cam.brightness = (55)'''
resolution = (64,64)
'stream = picamera.array.PiRGBArray(cam)'
for foo in cam.capture_continuous(stream, format='jpeg',use_video_port=True):
	'stream.truncate()'
	stream.seek(0)
	img = Image.open(stream)
	open_cv_image = np.array(img)
	open_cv_image = open_cv_image[:,:,::-1]
	DoStuff(open_cv_image)
	stream.seek(0)
	stream.truncate()

# if we are not using a video file, stop the camera video stream


# close all windows
cv2.destroyAllWindows()
