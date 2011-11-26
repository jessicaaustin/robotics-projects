#!/usr/bin/python

#
# Tracks a blue ball using OpenCV
#

import cv
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-c", "--camera", dest="camera_device", default=0,
                    help="the index of your camera. if /dev/videoN is your camera device, then --camera=N [default: %default]")
parser.add_option("-s", "--smoothness", dest="smoothness", default=4,
                    help="how many previous positions to interpolate to find our current position. higher smoothness => slower tracking, but less jerkiness [default: %default]")

(options, args) = parser.parse_args()
MY_CAMERA = int(options.camera_device)
SMOOTHNESS = int(options.smoothness)

# convert the given image to a binary image where all values are 
# zero other than areas with blue hue
def thresholded_image(image):
    # convert image to hsv
    image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
    cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)
    # threshold the image
    image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, 1)
    blue_min = cv.Scalar(70, 110, 120)
    blue_max = cv.Scalar(105, 255, 255)
    cv.InRangeS(image_hsv, blue_min, blue_max, image_threshed)
    return image_threshed

# initialize camera feed
capture = cv.CaptureFromCAM(MY_CAMERA)
if not capture:
    print "Could not initialize camera feed!"
    exit(1)

# create display windows
cv.NamedWindow('camera', cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow('threshed', cv.CV_WINDOW_AUTOSIZE)

# initialize position array
positions_x, positions_y = [0]*SMOOTHNESS, [0]*SMOOTHNESS

# read from the camera
print "Tracking ball... press any key to quit"
while 1:    
    image = cv.QueryFrame(capture)
    if not image:
        break

    # get the thresholded image
    image_threshed = thresholded_image(image)

    # finds the contours in our binary image
    contours = cv.FindContours(cv.CloneImage(image_threshed), cv.CreateMemStorage())
    # if there is a ball in the frame
    if len(contours) != 0:
        # calculate the moments to estimate the position of the ball
        moments = cv.Moments(contours, 1)
        moment10 = cv.GetSpatialMoment(moments, 1, 0)
        moment01 = cv.GetSpatialMoment(moments, 0, 1)
        area = cv.GetCentralMoment(moments, 0, 0)

        # if we got a good enough blob
        if area>0:
            positions_x.append(moment10/area)
            positions_y.append(moment01/area)
            # discard all but the last N positions
            positions_x, positions_y = positions_x[-SMOOTHNESS:], positions_y[-SMOOTHNESS:]
            print("pos",(positions_x[-1],positions_y[-1]))

    # show where the ball is located
    ball_indicator = cv.CreateImage(cv.GetSize(image), image.depth, 3)
    pos_x = (sum(positions_x)/len(positions_x))
    pos_y = (sum(positions_y)/len(positions_y))
    cv.Circle(ball_indicator, (int(pos_x),int(pos_y)), 8, (0,255,0), 2)
    cv.Add(image, ball_indicator, image)

    # show the images
    cv.ShowImage('threshed', image_threshed)
    cv.ShowImage('camera', image)

    # break from the loop if there is a key press
    c = cv.WaitKey(10)
    if c != -1:
        break

