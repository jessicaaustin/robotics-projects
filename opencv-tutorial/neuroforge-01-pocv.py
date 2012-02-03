#!/usr/bin/python

#
# OpenCV "Hello, world" 
# http://www.neuroforge.co.uk/index.php/getting-started-with-python-a-opencv
#
# loads an image from a file, adds some text, 
# then displays the image and saves it to a file

import cv

# create a window
cv.NamedWindow('a_window', cv.CV_WINDOW_AUTOSIZE)
# load an image from the filesystem
image = cv.LoadImage('picture.png', cv.CV_LOAD_IMAGE_COLOR)

# create some text using the given font, and place the text on the image
font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 3, 8)
x = 0
y = 60
cv.PutText(image, "Hello World!!!", (x,y), font, 255)

# put the image in the window
cv.ShowImage('a_window', image)
# show the window on the screen
cv.WaitKey(10000)
# save the image to the filesystem
cv.SaveImage('image.png', image)
