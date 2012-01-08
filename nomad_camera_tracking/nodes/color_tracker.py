#!/usr/bin/env python

"""

color-tracker.py

tracks a blob of a specified color
publishes to a topic (blob_coord) the x,y coords (in the world frame) of that blob

camera location: (.78, -.13, .115)
FOV: 88cm in x direction, 73cm in y direction when ball is 1 meter away
resolution: 640 x 480 pixels

6.92 pixels == 1 cm

"""

import roslib; roslib.load_manifest('nomad_camera_tracking')
import rospy

from geometry_msgs.msg import Point

import cv
import operator

import settings

# TODO parameterize whether or not we want to show the camera images (only in debug mode)
class ColorTracker():
    def __init__(self, camera_index, min_thresh, max_thresh, smoothness):
        # initialize camera feed
        self.capture = cv.CaptureFromCAM(camera_index)
        if not self.capture:
            err = "Could not init camera feed! camera_index=%s" % camera_index
            rospy.logerr(err)
            raise ROSInitException(err)

        # create display windows
        cv.NamedWindow('camera', cv.CV_WINDOW_AUTOSIZE)
#        cv.NamedWindow('threshed', cv.CV_WINDOW_AUTOSIZE)

        # store the image capture params
        self.smoothness = smoothness
        self.min_thresh = min_thresh
        self.max_thresh = max_thresh

        # initialize position array
        self.positions_x, self.positions_y = [0]*smoothness, [0]*smoothness

        rospy.loginfo("Successfully initialized ColorTracker")
    
    def find_blob(self):
        """
        captures a frame from the camera
        attempts to find the blob
        publishes the current blob location, if the blob was found
        """
        image = cv.QueryFrame(self.capture)
        if not image:
            rospy.logerr("Could not capture image")
            return None

        # smooth the image
        image_smoothed = cv.CloneImage(image)
        cv.Smooth(image, image_smoothed, cv.CV_GAUSSIAN, 15)
        # threshold the smoothed image
        image_threshed = self.thresholded_image(image_smoothed)
        
        # blobify
        cv.Dilate(image_threshed, image_threshed, None, 18)
        cv.Erode(image_threshed, image_threshed, None, 10)

        foundBlob = False

        # finds the contours in our binary image
        contours = cv.FindContours(cv.CloneImage(image_threshed), cv.CreateMemStorage())
        # if there is a matching object in the frame
        if len(contours) != 0:
            # calculate the moments to estimate the position of the object
            moments = cv.Moments(contours, 1)
            moment10 = cv.GetSpatialMoment(moments, 1, 0)
            moment01 = cv.GetSpatialMoment(moments, 0, 1)
            area = cv.GetCentralMoment(moments, 0, 0)

            # if we got a good enough blob
            if area>0:
                foundBlob = True
                self.positions_x.append(moment10/area)
                self.positions_y.append(moment01/area)
                # discard all but the last N positions
                self.positions_x, self.positions_y = self.positions_x[-self.smoothness:], self.positions_y[-self.smoothness:]

        # object_indicator will be the new image which shows where the identified
        # blob has been located.
        object_indicator = cv.CreateImage(cv.GetSize(image), image.depth, 3)

        # the average location of the identified blob
        pos_x = int(sum(self.positions_x)/len(self.positions_x))
        pos_y = int(sum(self.positions_y)/len(self.positions_y))
        object_position = (pos_x,pos_y)

        cv.Circle(object_indicator, object_position, 8, (0,255,0), 2)

        if foundBlob:
            # draw a line to the desiredPosition
            desiredPosition = cv.GetSize(image)
            desiredPosition = tuple(map(operator.mul, desiredPosition, (0.5, 0.5)))
            desiredPosition = tuple(map(int, desiredPosition))
            cv.Circle(object_indicator, desiredPosition, 4, (255,0,0), 2)
            cv.Line(object_indicator, object_position, desiredPosition, (255,0,0), 3) 

        # show the images
        cv.Add(image, object_indicator, image)
        cv.ShowImage('threshed', image_threshed)
        cv.ShowImage('camera', image)
        c = cv.WaitKey(1)

        if foundBlob:
            return Point(pos_x, pos_y, 0)
        else:
            return None


    def thresholded_image(self, image):
        """
        convert the given image to a binary image where all values are 
        zero other than areas with hue we're looking for
        """
        # convert image to hsv
        image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
        cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)
        # threshold the image
        image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, 1)
        cv.InRangeS(image_hsv, self.min_thresh, self.max_thresh, image_threshed)
        return image_threshed


if __name__ == '__main__':
    rospy.init_node('color_tracker')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing color_tracker node")
    color_tracker = ColorTracker(settings.MY_CAMERA, settings.MIN_THRESH, settings.MAX_THRESH, settings.SMOOTHNESS)
    pub_blob_coord = rospy.Publisher('blob_coord', Point)

    while not rospy.is_shutdown():
        blob_coord = color_tracker.find_blob()
        if blob_coord:
            pub_blob_coord.publish(blob_coord)
