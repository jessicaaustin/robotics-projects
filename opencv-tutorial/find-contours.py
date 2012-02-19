#!/usr/bin/python

#
# Finding image contours after thresholding
#
# load an image, convert to HSV color space, threshold the image
# for yellow hue values, then find contours 
#

import cv

# create windows for displaying our results 
cv.NamedWindow('original image', cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow('threshed image', cv.CV_WINDOW_AUTOSIZE)

# create our image from a file
#image = cv.LoadImage('yellow-ball.png', cv.CV_LOAD_IMAGE_COLOR)
#image = cv.LoadImage('yellow-ball-multiple-blobs.png', cv.CV_LOAD_IMAGE_COLOR)
image = cv.LoadImage('yellow-ball-multiple-blobs-flipped.png', cv.CV_LOAD_IMAGE_COLOR)

# convert the original image into HSV
image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)

# create the placeholder for thresholded image
channels = 1
image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, channels)

# do the actual thresholding
cv.InRangeS(image_hsv, cv.Scalar(20, 100, 100), cv.Scalar(30, 255, 255), image_threshed)
cv.Dilate(image_threshed, image_threshed, None, 18)
cv.Erode(image_threshed, image_threshed, None, 10)
cv.Canny(image_threshed, image_threshed, 10, 50, 3) 

contours = cv.FindContours(cv.CloneImage(image_threshed), cv.CreateMemStorage(), cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
for c in contours:
    print c
    cv.Circle(image, c, 1, (0,255,0), 1)

moments = cv.Moments(contours, 1)
center = (cv.GetSpatialMoment(moments, 1, 0)/cv.GetSpatialMoment(moments, 0, 0),cv.GetSpatialMoment(moments, 0, 1)/cv.GetSpatialMoment(moments, 0, 0))
cv.Circle(image, (int(center[0]), int(center[1])), 2, (0,0,255), 2)

# show the images
print "displaying images [press any key to continue]"
cv.ShowImage('original image', image)
cv.ShowImage('threshed image', image_threshed)
cv.WaitKey(10000)


