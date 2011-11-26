#!/usr/bin/python

#
# Tracking a colored blob in OpenCV
# http://www.aishack.in/2010/07/tracking-colored-objects-in-opencv/
#
#

import cv

def thresholded_image(image):
    # convert image to hsv
    image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
    cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)
    # threshold the image (note--I am tracking a blue ball, not a yellow one)
    image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, 1)
    blue_min = cv.Scalar(105, 200, 100)
    blue_max = cv.Scalar(115, 255, 255)
    cv.InRangeS(image_hsv, blue_min, blue_max, image_threshed)
    return image_threshed

# use this function to test your thresholds
def test_thresholded_image():
    capture = cv.CaptureFromCAM(0)
    image = cv.QueryFrame(capture)
    image_threshed = thresholded_image(image)
    cv.NamedWindow('test', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('test', 100, 100)
    cv.ShowImage('test', image_threshed)
    cv.WaitKey(1000)
    exit(0)

#test_thresholded_image()

# initialize camera feed
capture = cv.CaptureFromCAM(0)
if not capture:
    print "Could not initialize camera feed!"
    exit(1)

# create display windows
cv.NamedWindow('camera', cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow('threshed', cv.CV_WINDOW_AUTOSIZE)

# holds the tracked position of the ball
image_scribble = None
# the position of the ball
pos_x = 0
pos_y = 0

# read from the camera
print "Tracking ball... press any key to quit"
while 1:    
    image = cv.QueryFrame(capture)
    if not image:
        break

    # if this is the first frame, we need to initialize it
    if not image_scribble:
        image_scribble = cv.CreateImage(cv.GetSize(image), image.depth, 3)

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
            last_x = pos_x
            last_y = pos_y
            pos_x = moment10/area
            pos_y = moment01/area

            print("pos=(%s,%s)"%(pos_x,pos_y))

            # draw the tracking line
            if last_x>0 and last_y>0 and pos_x>0 and pos_y>0:
                pt1 = (int(last_x), int(last_y))
                pt2 = (int(pos_x), int(pos_y))
                cv.Line(image_scribble, pt1, pt2, (0, 255, 255), 5)

    # add the scribble to the original frame
    cv.Add(image, image_scribble, image)
    cv.ShowImage('threshed', image_threshed)
    cv.ShowImage('camera', image)

    # break from the loop if there is a key press
    c = cv.WaitKey(10)
    if c != -1:
        break

