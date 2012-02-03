#!/usr/bin/python

#
# Color thresholds in OpenCV
# http://www.aishack.in/2010/07/tracking-colored-objects-in-opencv/
#
# load an image, convert to HSV color space, and threshold the image
# for yellow hue values 
#

import cv

# create windows for displaying our results 
cv.NamedWindow('original image', cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow('threshed image', cv.CV_WINDOW_AUTOSIZE)

# create our image from a file
image = cv.LoadImage('yellow-ball.png', cv.CV_LOAD_IMAGE_COLOR)

# convert the original image into HSV
image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)

# create the placeholder for thresholded image
channels = 1
image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, channels)

# do the actual thresholding
cv.InRangeS(image_hsv, cv.Scalar(20, 100, 100), cv.Scalar(30, 255, 255), image_threshed)

# show the images
print "displaying images [press any key to continue]"
cv.ShowImage('original image', image)
cv.ShowImage('threshed image', image_threshed)
cv.WaitKey(10000)
