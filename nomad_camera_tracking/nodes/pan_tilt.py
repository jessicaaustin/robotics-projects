#!/usr/bin/env python

"""
pan_tilt.py

subscribes to the source of color blob location coordinates
and updates the aim of the camera, based on those coordinates.
"""

import roslib ; roslib.load_manifest('nomad_camera_tracking')
import rospy

from geometry_msgs.msg import Point
from PanTilt import PanTilt

panTilt = None

def updateBlobLocation(coordinateUpdateMessage):
    # adjust the camera
    print "Point message received"
    print "x: %f, y:%f, z: %f" % (coordinateUpdateMessage.x, coordinateUpdateMessage.y, coordinateUpdateMessage.z)
    print "\n"

    return

def mainControlLoop():

	global panTilt

    blobCoordSubscriber = rospy.Subscriber('blob_coord', Point, updateBlobLocation)
	panTilt = PanTilt('/dev/ttyUSB0')
	panTilt.centerCamera()

    while not rospy.is_shutdown():
        rospy.spin()

    return

if __name__ == '__main__':

    rospy.init_node('pan_tilt', log_level = rospy.DEBUG)
    rospy.loginfo('pan_tilt starting')

    mainControlLoop()

    rospy.logwarn('Stopping')

    sys.exit(0)

