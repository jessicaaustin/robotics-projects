#!/usr/bin/env python

"""
pan_tilt.py

subscribes to the source of color blob location coordinates
and updates the aim of the camera, based on those coordinates.
"""

import roslib ; roslib.load_manifest('nomad_camera_tracking')
import rospy
import random
import sys

from geometry_msgs.msg import Point
from PanTilt import PanTilt

panTilt = None

def mainControlLoop():

    global panTilt

    panTilt = PanTilt('/dev/ttyUSB0')
    panTilt.setSpeed(1000)
    panTilt.centerCamera()
    cameraAim = Point()
    cameraAim.z = 0

    random.seed()

#    blobTracker = rospy.Subscriber('', Point, callback)
    trackingCamera = rospy.Publisher('trackingCamera', Point)
    
    while not rospy.is_shutdown():

        randomPan = random.randint(-10, 10)
        randomTilt = random.randint(-10, 10)
    
        panTilt.aimCamera(randomPan, randomTilt)
    
        cameraAim.x, cameraAim.y = panTilt.getCameraAim()
        print cameraAim.x, cameraAim.y
        trackingCamera.publish(cameraAim)

        rospy.sleep(0.5)

    return

if __name__ == '__main__':

    rospy.init_node('pan_tilt', log_level = rospy.DEBUG)
    rospy.loginfo('pan_tilt starting')

    mainControlLoop()

    rospy.logwarn('Stopping')

    sys.exit(0)

