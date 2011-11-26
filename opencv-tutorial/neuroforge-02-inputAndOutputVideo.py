#!/usr/bin/python

#
# Input and output with OpenCV
# http://www.neuroforge.co.uk/index.php/input-and-output-with-open-cv
#
# reads images from a camera and displays them, then reads a 
# sequence of images and writes them to an avi file
#

import cv

# connect to the camera. 0 refers to which camera (look for /dev/videoN)
capture = cv.CaptureFromCAM(0)
if not capture:
    print "could not capture from camera!"
    exit(1)

# grab a frame and display it
print "displaying camera image [press any key to continue]"
image = cv.QueryFrame(capture)
cv.NamedWindow('camera', cv.CV_WINDOW_AUTOSIZE)
cv.ShowImage('camera', image)
cv.WaitKey(10000)

# create a video writer
writer = cv.CreateVideoWriter("output.avi", 0, 15, cv.GetSize(image), 1)

# capture a series of images and write to the file
count = 0 
while count<250:
    image = cv.QueryFrame(capture)
    cv.WriteFrame(writer, image)
    cv.ShowImage('camera', image)
    cv.WaitKey(2)
    count+=1

