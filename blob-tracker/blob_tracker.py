#!/usr/bin/python

#
# Tracks a blue ball using OpenCV
#

import cv
from optparse import OptionParser

# TODO use argparse instead
parser = OptionParser()
parser.add_option("-c", "--camera", dest="camera_device", default=0,
                    help="the index of your camera. if /dev/videoN is your camera device, then --camera=N [default: %default]")
parser.add_option("-m", "--min-threshold", dest="min_threshold",
                    default="(70, 110, 120)",
                    help="minimum threshold value in HSV (hue,sat,val). if not given, will track a blue ball. [default: %default]")
parser.add_option("-x", "--max-threshold", dest="max_threshold",
                    default="(105, 255, 255)",
                    help="maximum threshold value in HSV (hue,sat,val). if not given, will track a blue ball. [default: %default]")
parser.add_option("-s", "--smoothness", dest="smoothness", default=4,
                    help="how many previous positions to interpolate to find our current position. higher smoothness => slower tracking, but less jerkiness [default: %default]")
parser.add_option("", "--red", action="store_true", dest="track_red", help="track a red object")
parser.add_option("", "--green", action="store_true", dest="track_green", help="track a green object")
parser.add_option("", "--blue", action="store_true", dest="track_blue", help="track a blue object")

(options, args) = parser.parse_args()
MY_CAMERA = int(options.camera_device)
SMOOTHNESS = int(options.smoothness)
MIN_THRESH = eval(options.min_threshold)
MAX_THRESH = eval(options.max_threshold)

if options.track_red:
    MIN_THRESH, MAX_THRESH = (163.0, 85.5, 72.5, 0.0), (189.0, 244.5, 247.5, 0.0)
if options.track_green:
    MIN_THRESH, MAX_THRESH = ( 60.5, 74.5, 73.5, 0.0), (109.5, 215.5, 206.5, 0.0)
if options.track_blue:
    MIN_THRESH, MAX_THRESH = ( 75.0, 80.0, 80.0, 0.0), (125.0, 230.0, 230.0, 0.0)

# convert the given image to a binary image where all values are 
# zero other than areas with blue hue
def thresholded_image(image):
    # convert image to hsv
    image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
    cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)
    # threshold the image
    image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, 1)
    cv.InRangeS(image_hsv, MIN_THRESH, MAX_THRESH, image_threshed)
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
#            print("pos",(positions_x[-1],positions_y[-1]))

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

