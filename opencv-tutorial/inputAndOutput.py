#!/usr/bin/python

#
# Input and output with OpenCV
# http://www.neuroforge.co.uk/index.php/input-and-output-with-open-cv
#
# Reads an image from disk, copies part of that image into another image,
# draws some shapes, and saves the new image
#

import cv

# create windows for displaying our results 
cv.NamedWindow('window1', cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow('window2', cv.CV_WINDOW_AUTOSIZE)

# create our first image from a file
capture = cv.CaptureFromFile('picture.png')
image = cv.QueryFrame(capture)

# create a second image -- this one is empty
empty_image = cv.CloneImage(image)
cv.Zero(empty_image)

# display both images
print "showing original images (press any key to continue)"
cv.ShowImage('window1', image)
cv.ShowImage('window2', empty_image)
cv.WaitKey(10000)

# copy some pixels from the original image to the second one
(minx,miny) = (150,100)
(maxx,maxy) = (300,250)
for x in range(minx,maxx):
    for y in range(miny,maxy):
        value = cv.Get2D(image,y,x)
        cv.Set2D(empty_image,y,x,value)

# display both images
print "showing original image plus a copied section (press any key to continue)"
cv.ShowImage('window1', image)
cv.ShowImage('window2', empty_image)
cv.WaitKey(10000)

# draw a blue rectangle around the copied section
cv.Rectangle(empty_image, (minx,miny), (maxx,maxy), (255,0,0), 4)
# draw green circles on the vertices
cv.Circle(empty_image, (minx,miny), 8, (0,255,0), 2)
cv.Circle(empty_image, (maxx,miny), 8, (0,255,0), 2)
cv.Circle(empty_image, (minx,maxy), 8, (0,255,0), 2)
cv.Circle(empty_image, (maxx,maxy), 8, (0,255,0), 2)

# display both images
print "showing original image plus a copied section with shapes (press any key to continue)"
cv.ShowImage('window1', image)
cv.ShowImage('window2', empty_image)
cv.WaitKey(10000)

# save the new image to the filesystem
cv.SaveImage('image.png', empty_image)
