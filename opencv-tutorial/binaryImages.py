#!/usr/bin/python

#
# Binary images with OpenCV
# http://www.neuroforge.co.uk/index.php/binary-images-with-opencv
#
# reads an image from a file, converts it to greyscale, and then
# applies various filters to the image
#

import cv

# create a window for displaying our results 
cv.NamedWindow('window', cv.CV_WINDOW_AUTOSIZE)
cv.MoveWindow('window', 100, 100)

# create our image from a file
image = cv.LoadImage('picture.png', cv.CV_LOAD_IMAGE_COLOR)

# helper function to show our image
def showWindow(message):
    print message
    print "  [press any key to continue]"
    cv.ShowImage('window', image)
    cv.WaitKey(10000)

showWindow("showing original image")

# helper function to load greyscale version of our image
def loadGreyscale():
    return cv.LoadImage('picture.png', cv.CV_LOAD_IMAGE_GRAYSCALE)

image = loadGreyscale()
showWindow("showing greyscale")

# smooth the image
image = loadGreyscale()
cv.Smooth(image, image, cv.CV_MEDIAN)
showWindow("smoothed image")

# histogram
image = loadGreyscale()
cv.EqualizeHist(image, image)
showWindow("histogram")

# binary threshold
image = loadGreyscale()
threshold = 100
color = 255
cv.Threshold(image, image, threshold, color, cv.CV_THRESH_BINARY)
showWindow("binary threshold")

# Otsu threshold
image = loadGreyscale()
cv.Threshold(image, image, threshold, color, cv.CV_THRESH_OTSU)
showWindow("Otsu threshold")

# Dilation
image = loadGreyscale()
element_shape = cv.CV_SHAPE_RECT
pos = 1
element = cv.CreateStructuringElementEx(pos*2+1, pos*2+1, pos, pos, element_shape)
cv.Dilate(image, image, element, 2)
showWindow("Dilate")

# Erosion
image = loadGreyscale()
cv.Erode(image, image, element, 2)
showWindow("Erode")

# Morphology
image = loadGreyscale()
cv.MorphologyEx(image, image, image, element, cv.CV_MOP_CLOSE, 2)
showWindow("Morphology")

# Laplace
image = loadGreyscale()
dst_16s2 = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_16S, 1)
cv.Laplace(image, dst_16s2)
cv.Convert(dst_16s2, image)
showWindow('Laplace')


