#!/usr/bin/python

import cv

cv.NamedWindow('a_window', cv.CV_WINDOW_AUTOSIZE)
image = cv.LoadImage('picture.png', cv.CV_LOAD_IMAGE_COLOR)

font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 3, 8)
x = 0
y = 60
cv.PutText(image, "Hello World!!!", (x,y), font, 255)

cv.ShowImage('a_window', image)
cv.WaitKey(10000)
cv.SaveImage('image.png', image)
