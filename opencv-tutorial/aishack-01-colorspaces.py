#!/usr/bin/python

#
# Color spaces in OpenCV
# http://www.aishack.in/2010/01/color-spaces/
#
# demonstrates conversions in RGB and HSV color spaces
#

import cv

# create a window for displaying our results 
cv.NamedWindow('window', cv.CV_WINDOW_AUTOSIZE)
cv.MoveWindow('window', 100, 100)

# create our image from a file
image = cv.LoadImage('colorspace.png', cv.CV_LOAD_IMAGE_COLOR)

# helper function to show our image
def showWindow(message):
    print message
    print "  [press any key to continue]"
    font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5)
    cv.PutText(image, message, (10,20), font, cv.RGB(150,150,150))
    cv.ShowImage('window', image)
    cv.WaitKey(10000)

# create the r,g,b images placeholders
red = cv.CreateImage(cv.GetSize(image), image.depth, 1)
cv.SetZero(red)
green = cv.CloneImage(red)
blue = cv.CloneImage(red)

# split the original image into its channels
cv.Split(image, blue, green, red, None)

# show the channels
showWindow("original (RGB)")
image = red
showWindow("red")
image = green
showWindow("green")
image = blue
showWindow("blue")

# reload original image
image = cv.LoadImage('colorspace.png', cv.CV_LOAD_IMAGE_COLOR)

# create the hue, sat, value image placeholders
hue = cv.CreateImage(cv.GetSize(image), image.depth, 1)
cv.SetZero(hue)
sat = cv.CloneImage(hue)
val = cv.CloneImage(hue)

# convert the original image into HSV
hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
cv.CvtColor(image, hsv, cv.CV_BGR2HSV)

# split the HSV image into the channels
cv.Split(hsv, hue, sat, val, None)

# show the images
image = hsv
showWindow("hsv image")
image = hue
showWindow("hue")
image = sat
showWindow("sat")
image = val
showWindow("val")
